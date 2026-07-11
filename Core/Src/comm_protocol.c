#include "comm_protocol.h"
#include "advance_chassis.h"
#include "advance_motion.h"
#include "advance_world.h"
#include "advance_arm.h"
#include <stdbool.h>
#include <string.h>

/*
 * 上位机主控协议说明
 *
 * 当前协议面向 Jetson / Windows 上位机，底层默认从 USART6 接收。
 * PC 调试口 USART1 也可注册进同一解析器，用于串口助手发帧测试。
 *
 * 基础帧格式：
 *   5A A5 Version MsgType CmdSet CmdID Seq_L Seq_H Length Payload CRC_L CRC_H
 *
 * 关键约定：
 * - Header 固定为 0x5A 0xA5。
 * - Version 当前固定为 0x01。
 * - Seq、Payload 内的多字节整数均为小端。
 * - CRC 使用 CRC16-Modbus，从 Version 字节开始计算，到 Payload 最后一个字节结束。
 * - 中断 / DMA 回调只负责喂入字节流，命令执行统一放在 HostProtocol_Poll()。
 */
#define comm_protocol_HEADER1 ((uint8_t)0x5AU)
#define comm_protocol_HEADER2 ((uint8_t)0xA5U)
#define comm_protocol_VERSION ((uint8_t)0x01U)
#define comm_protocol_HEADER_LEN ((uint16_t)9U)
#define comm_protocol_CRC_LEN ((uint16_t)2U)
#define comm_protocol_MAX_PAYLOAD ((uint16_t)255U)
#define comm_protocol_MAX_FRAME_LEN (comm_protocol_HEADER_LEN + comm_protocol_MAX_PAYLOAD + comm_protocol_CRC_LEN)
#define comm_protocol_QUEUE_SIZE ((uint8_t)4U)
#define comm_protocol_TX_BITS_PER_BYTE ((uint32_t)10U)
#define comm_protocol_TX_TIMEOUT_GUARD_MS ((uint32_t)5U)
#define comm_protocol_TX_MAX_RETRIES ((uint8_t)1U)
#define comm_protocol_HEARTBEAT_MS ((uint32_t)300U)
#define comm_protocol_TX_QUEUE_SIZE ((uint8_t)8U)

typedef enum
{
  /* 上位机发给 STM32 的控制命令。 */
  MSG_CMD = 0x01,
  /* STM32 对命令接收和执行结果的确认。 */
  MSG_ACK = 0x02,
  MSG_DATA = 0x03
} HostProtocol_MsgType_t;

typedef enum
{
  /* 通信测试、心跳、模式设置。 */
  CMDSET_SYSTEM = 0x01,
  /* 急停、安全停止、安全状态清除。 */
  CMDSET_SAFETY = 0x02,
  /* 底盘使能、停止、四轮 RPM、麦克纳姆合成速度。 */
  CMDSET_CHASSIS = 0x03,
  /* 机械臂与夹爪等上层执行器动作。 */
  CMDSET_SERVO = 0x04
} HostProtocol_CmdSet_t;

typedef enum
{
  /* result 字段会放入 ACK Payload[2]，上位机据此判断命令是否执行。 */
  ACK_OK = 0x00,
  ACK_BAD_CRC = 0x01,
  ACK_BAD_LENGTH = 0x02,
  ACK_BAD_PARAM = 0x03,
  ACK_BUSY = 0x04,
  ACK_DENIED = 0x05,
  ACK_UNKNOWN_CMD = 0x06,
  ACK_FAULT = 0x07
} HostProtocol_AckResult_t;

typedef struct
{
  /* 正在收集的一帧原始数据，包含帧头和 CRC。 */
  uint8_t frame[comm_protocol_MAX_FRAME_LEN];
  /* 当前已经收到的字节数。 */
  uint16_t pos;
  /* 收到 Length 字段后才能确定完整帧总长。 */
  uint16_t expected_len;
} HostProtocol_Parser_t;

typedef struct
{
  HostProtocol_Source_t source;
  uint8_t msg_type;
  uint8_t cmd_set;
  uint8_t cmd_id;
  uint16_t seq;
  uint8_t payload_len;
  uint8_t payload[comm_protocol_MAX_PAYLOAD];
} HostProtocol_Frame_t;

typedef struct
{
  UART_HandleTypeDef *huart;
  uint16_t length;
  uint8_t retry_count;
  uint8_t data[comm_protocol_MAX_FRAME_LEN];
} HostProtocol_TxItem_t;

static UART_HandleTypeDef *g_comm_protocol_uart[comm_protocol_SOURCE_COUNT] = {0};
static HostProtocol_Parser_t g_comm_protocol_parser[comm_protocol_SOURCE_COUNT] = {0};
static volatile uint8_t g_queue_head = 0U;
static volatile uint8_t g_queue_tail = 0U;
static volatile uint8_t g_queue_count = 0U;
static HostProtocol_Frame_t g_queue[comm_protocol_QUEUE_SIZE] = {0};
static HostProtocol_Status_t g_last_status = comm_protocol_STATUS_OK;
static uint32_t g_last_heartbeat_tick = 0U;
static uint8_t g_heartbeat_seen = 0U;
static uint8_t g_heartbeat_online = 0U;
static uint8_t g_safety_latched = 0U;
static uint8_t g_control_mode = 0U;
static HostProtocol_TxItem_t g_tx_queue[comm_protocol_TX_QUEUE_SIZE] = {0};
static volatile uint8_t g_tx_head = 0U;
static volatile uint8_t g_tx_tail = 0U;
static volatile uint8_t g_tx_count = 0U;
static volatile uint8_t g_tx_active = 0U;
static volatile uint32_t g_tx_started_tick = 0U;
static volatile uint32_t g_tx_timeout_count = 0U;

typedef enum
{
  HOST_CONTROL_IDLE = 0U,
  HOST_CONTROL_MOTOR_RPM,
  HOST_CONTROL_MECANUM,
  HOST_CONTROL_BODY_VELOCITY,
  HOST_CONTROL_WORLD_VELOCITY,
  HOST_CONTROL_GOTO_POSE,
  HOST_CONTROL_SAFETY_LOCKED
} HostProtocol_ControlMode_t;

static void HostProtocol_StopMotion(uint8_t immediate)
{
  AdvanceMotion_CancelIfActive();
  if (immediate != 0U)
  {
    Chassis_Stop();
  }
  else
  {
    Chassis_SmoothStop(CHASSIS_DEFAULT_ACC);
  }
  g_control_mode = HOST_CONTROL_IDLE;
}

static void HostProtocol_SelectMotionMode(HostProtocol_ControlMode_t mode)
{
  if (mode != HOST_CONTROL_GOTO_POSE)
  {
    AdvanceMotion_CancelIfActive();
  }
  g_control_mode = (uint8_t)mode;
}

static uint32_t HostProtocol_GetTxTimeoutMs(const HostProtocol_TxItem_t *item)
{
  uint32_t baud_rate;
  uint32_t transmit_ms;

  if ((item == NULL) || (item->huart == NULL) || (item->huart->Init.BaudRate == 0U))
  {
    return comm_protocol_TX_TIMEOUT_GUARD_MS;
  }

  baud_rate = item->huart->Init.BaudRate;
  transmit_ms = (((uint32_t)item->length * comm_protocol_TX_BITS_PER_BYTE * 1000U) +
                 baud_rate - 1U) /
                baud_rate;
  return transmit_ms + comm_protocol_TX_TIMEOUT_GUARD_MS;
}

static void HostProtocol_StartNextTx(void)
{
  HAL_StatusTypeDef status;

  if ((g_tx_active != 0U) || (g_tx_count == 0U))
  {
    return;
  }

  g_tx_active = 1U;
  status = HAL_UART_Transmit_IT(g_tx_queue[g_tx_tail].huart,
                                g_tx_queue[g_tx_tail].data,
                                g_tx_queue[g_tx_tail].length);
  if (status == HAL_OK)
  {
    g_tx_started_tick = HAL_GetTick();
  }
  else
  {
    g_tx_active = 0U;
    g_last_status = comm_protocol_STATUS_OVERFLOW;
  }
}

static void HostProtocol_RecoverTxTimeout(void)
{
  HostProtocol_TxItem_t *item;
  uint32_t now;

  now = HAL_GetTick();
  __disable_irq();
  if ((g_tx_active == 0U) || (g_tx_count == 0U))
  {
    __enable_irq();
    return;
  }

  item = &g_tx_queue[g_tx_tail];
  if ((now - g_tx_started_tick) <= HostProtocol_GetTxTimeoutMs(item))
  {
    __enable_irq();
    return;
  }

  /* 当前发送使用中断模式，AbortTransmit 会停止 TX 中断并恢复 UART READY 状态。 */
  (void)HAL_UART_AbortTransmit(item->huart);
  ++g_tx_timeout_count;
  g_last_status = comm_protocol_STATUS_TX_TIMEOUT;
  g_tx_active = 0U;
  g_tx_started_tick = 0U;
  if (item->retry_count < comm_protocol_TX_MAX_RETRIES)
  {
    ++item->retry_count;
  }
  else
  {
    g_tx_tail = (uint8_t)((g_tx_tail + 1U) % comm_protocol_TX_QUEUE_SIZE);
    --g_tx_count;
  }
  __enable_irq();

  HostProtocol_StartNextTx();
}

static void HostProtocol_QueueTx(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t length)
{
  uint8_t slot;

  if ((huart == NULL) || (data == NULL) || (length == 0U) ||
      (length > comm_protocol_MAX_FRAME_LEN))
  {
    g_last_status = comm_protocol_STATUS_INVALID_PARAM;
    return;
  }

  __disable_irq();
  if (g_tx_count >= comm_protocol_TX_QUEUE_SIZE)
  {
    __enable_irq();
    g_last_status = comm_protocol_STATUS_OVERFLOW;
    return;
  }
  slot = g_tx_head;
  g_tx_queue[slot].huart = huart;
  g_tx_queue[slot].length = length;
  g_tx_queue[slot].retry_count = 0U;
  memcpy(g_tx_queue[slot].data, data, length);
  g_tx_head = (uint8_t)((slot + 1U) % comm_protocol_TX_QUEUE_SIZE);
  ++g_tx_count;
  __enable_irq();

  HostProtocol_StartNextTx();
}

/* 协议中 uint16_t 使用小端：低字节在前，高字节在后。 */
static uint16_t HostProtocol_ReadU16(const uint8_t *data)
{
  return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static int16_t HostProtocol_ReadI16(const uint8_t *data)
{
  return (int16_t)HostProtocol_ReadU16(data);
}

static uint32_t HostProtocol_ReadU32(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}

static int32_t HostProtocol_ReadI32(const uint8_t *data)
{
  return (int32_t)HostProtocol_ReadU32(data);
}

static void HostProtocol_WriteU16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void HostProtocol_WriteU32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8) & 0xFFU);
  data[2] = (uint8_t)((value >> 16) & 0xFFU);
  data[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static void HostProtocol_WriteI32(uint8_t *data, int32_t value)
{
  HostProtocol_WriteU32(data, (uint32_t)value);
}

static int32_t HostProtocol_FloatToI32(float value)
{
  return (value >= 0.0f) ? (int32_t)(value + 0.5f) : (int32_t)(value - 0.5f);
}

/*
 * CRC16-Modbus 计算。
 *
 * 调试时注意：
 * - 初值 0xFFFF，多项式 0xA001。
 * - 输入 data 必须从 frame[2] 也就是 Version 开始。
 * - length = 7 + payload_len，对应 Version..Length..Payload。
 * - 返回值发送时仍按小端拆成 CRC_L、CRC_H。
 */
static uint16_t HostProtocol_Crc16Modbus(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  uint16_t index;
  uint8_t bit;

  for (index = 0U; index < length; ++index)
  {
    crc ^= data[index];
    for (bit = 0U; bit < 8U; ++bit)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (uint16_t)((crc >> 1) ^ 0xA001U);
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}

static void HostProtocol_ResetParser(HostProtocol_Parser_t *parser)
{
  parser->pos = 0U;
  parser->expected_len = 0U;
}

static uint8_t HostProtocol_AbsRpmTooLarge(int16_t rpm)
{
  int32_t value = rpm;

  if (value < 0)
  {
    value = -value;
  }

  return (value > (int32_t)CHASSIS_MAX_RPM) ? 1U : 0U;
}

static uint8_t HostProtocol_AbsI16Exceeds(int16_t value, int32_t limit)
{
  int32_t signed_value = value;

  if (signed_value < 0)
  {
    signed_value = -signed_value;
  }
  return (signed_value > limit) ? 1U : 0U;
}

/*
 * 普通底盘命令的安全闸门。
 *
 * 急停锁定后，或者已经启用过心跳但心跳超时后，普通运动命令会被拒绝。
 * SAFETY 命令本身不走这个检查，保证急停/清除等动作仍可执行。
 */
static uint8_t HostProtocol_IsControlAllowed(void)
{
  if (g_safety_latched != 0U)
  {
    return 0U;
  }

  if ((g_heartbeat_seen != 0U) && (g_heartbeat_online == 0U))
  {
    return 0U;
  }

  return 1U;
}

/* 收到过心跳后，如果超过 300ms 未刷新，则认为上位机离线并停车。 */
static void HostProtocol_CheckHeartbeatTimeout(void)
{
  if ((g_heartbeat_seen != 0U) &&
      (g_heartbeat_online != 0U) &&
      ((HAL_GetTick() - g_last_heartbeat_tick) > comm_protocol_HEARTBEAT_MS))
  {
    g_heartbeat_online = 0U;
    HostProtocol_StopMotion(1U);
  }
}

/*
 * 把校验通过的完整帧转成轻量命令对象，放入主循环队列。
 *
 * 这样 DMA/IDLE 回调不会直接调用电机、舵机等业务函数，降低中断内耗时
 * 和串口收包期间的竞态风险。
 */
static uint8_t HostProtocol_EnqueueFrame(HostProtocol_Source_t source, const uint8_t *frame, uint16_t frame_len)
{
  HostProtocol_Frame_t *item;
  uint8_t payload_len;

  if ((source >= comm_protocol_SOURCE_COUNT) || (frame_len < (comm_protocol_HEADER_LEN + comm_protocol_CRC_LEN)))
  {
    g_last_status = comm_protocol_STATUS_INVALID_PARAM;
    return 0U;
  }

  if (g_queue_count >= comm_protocol_QUEUE_SIZE)
  {
    g_last_status = comm_protocol_STATUS_OVERFLOW;
    return 0U;
  }

  payload_len = frame[8];
  item = &g_queue[g_queue_head];
  item->source = source;
  item->msg_type = frame[3];
  item->cmd_set = frame[4];
  item->cmd_id = frame[5];
  item->seq = HostProtocol_ReadU16(&frame[6]);
  item->payload_len = payload_len;
  if (payload_len > 0U)
  {
    memcpy(item->payload, &frame[9], payload_len);
  }

  g_queue_head = (uint8_t)((g_queue_head + 1U) % comm_protocol_QUEUE_SIZE);
  ++g_queue_count;
  g_last_status = comm_protocol_STATUS_OK;
  return 1U;
}

/* 主循环取命令时短暂关中断，避免和接收回调同时修改队列索引。 */
static uint8_t HostProtocol_DequeueFrame(HostProtocol_Frame_t *frame)
{
  if (frame == NULL)
  {
    return 0U;
  }

  __disable_irq();
  if (g_queue_count == 0U)
  {
    __enable_irq();
    return 0U;
  }

  memcpy(frame, &g_queue[g_queue_tail], sizeof(HostProtocol_Frame_t));
  g_queue_tail = (uint8_t)((g_queue_tail + 1U) % comm_protocol_QUEUE_SIZE);
  --g_queue_count;
  __enable_irq();
  return 1U;
}

/*
 * 回发 ACK。
 *
 * ACK 帧使用原命令的 CmdSet / CmdID / Seq，Payload 固定 4 字节：
 *   ack_seq_l ack_seq_h result detail
 *
 * 上位机调试时只要按 Seq 匹配 ACK，就能判断哪条命令成功或失败。
 */
static void HostProtocol_SendAck(const HostProtocol_Frame_t *frame, HostProtocol_AckResult_t result, uint8_t detail)
{
  UART_HandleTypeDef *huart;
  uint8_t tx[comm_protocol_HEADER_LEN + 4U + comm_protocol_CRC_LEN];
  uint16_t crc;

  if ((frame == NULL) || (frame->source >= comm_protocol_SOURCE_COUNT))
  {
    return;
  }

  huart = g_comm_protocol_uart[frame->source];
  if (huart == NULL)
  {
    return;
  }

  tx[0] = comm_protocol_HEADER1;
  tx[1] = comm_protocol_HEADER2;
  tx[2] = comm_protocol_VERSION;
  tx[3] = MSG_ACK;
  tx[4] = frame->cmd_set;
  tx[5] = frame->cmd_id;
  HostProtocol_WriteU16(&tx[6], frame->seq);
  tx[8] = 4U;
  HostProtocol_WriteU16(&tx[9], frame->seq);
  tx[11] = (uint8_t)result;
  tx[12] = detail;
  crc = HostProtocol_Crc16Modbus(&tx[2], 7U + tx[8]);
  HostProtocol_WriteU16(&tx[13], crc);

  HostProtocol_QueueTx(huart, tx, (uint16_t)sizeof(tx));
}

static void HostProtocol_SendData(const HostProtocol_Frame_t *frame, const uint8_t *payload, uint8_t payload_len)
{
  UART_HandleTypeDef *huart;
  uint8_t tx[comm_protocol_HEADER_LEN + comm_protocol_MAX_PAYLOAD + comm_protocol_CRC_LEN];
  uint16_t crc;

  if ((frame == NULL) || (frame->source >= comm_protocol_SOURCE_COUNT))
  {
    return;
  }

  huart = g_comm_protocol_uart[frame->source];
  if (huart == NULL)
  {
    return;
  }

  tx[0] = comm_protocol_HEADER1;
  tx[1] = comm_protocol_HEADER2;
  tx[2] = comm_protocol_VERSION;
  tx[3] = MSG_DATA;
  tx[4] = frame->cmd_set;
  tx[5] = frame->cmd_id;
  HostProtocol_WriteU16(&tx[6], frame->seq);
  tx[8] = payload_len;
  if ((payload != NULL) && (payload_len > 0U))
  {
    memcpy(&tx[9], payload, payload_len);
  }
  crc = HostProtocol_Crc16Modbus(&tx[2], (uint16_t)(7U + payload_len));
  HostProtocol_WriteU16(&tx[9U + payload_len], crc);

  HostProtocol_QueueTx(huart, tx, (uint16_t)(comm_protocol_HEADER_LEN + payload_len + comm_protocol_CRC_LEN));
}

static void HostProtocol_SendMotionStatusData(const HostProtocol_Frame_t *frame)
{
  AdvanceMotion_RuntimeStatus_t status;
  uint8_t payload[56];
  uint32_t now;
  uint32_t elapsed_ms = 0U;

  if (AdvanceMotion_GetStatus(&status) != ADVANCE_MOTION_STATUS_OK)
  {
    return;
  }

  now = HAL_GetTick();
  if (status.started_tick != 0U)
  {
    elapsed_ms = now - status.started_tick;
  }

  memset(payload, 0, sizeof(payload));
  payload[0] = (uint8_t)status.state;
  payload[1] = status.active;
  payload[2] = status.goal.goal_flags;
  payload[3] = 0U;
  HostProtocol_WriteI32(&payload[4], HostProtocol_FloatToI32(status.pose.x_mm));
  HostProtocol_WriteI32(&payload[8], HostProtocol_FloatToI32(status.pose.y_mm));
  HostProtocol_WriteI32(&payload[12], HostProtocol_FloatToI32(status.pose.yaw_deg * 100.0f));
  HostProtocol_WriteI32(&payload[16], HostProtocol_FloatToI32(status.error_x_mm));
  HostProtocol_WriteI32(&payload[20], HostProtocol_FloatToI32(status.error_y_mm));
  HostProtocol_WriteI32(&payload[24], HostProtocol_FloatToI32(status.position_error_mm));
  HostProtocol_WriteI32(&payload[28], HostProtocol_FloatToI32(status.yaw_error_deg * 100.0f));
  HostProtocol_WriteI32(&payload[32], HostProtocol_FloatToI32(status.goal.x_mm));
  HostProtocol_WriteI32(&payload[36], HostProtocol_FloatToI32(status.goal.y_mm));
  HostProtocol_WriteI32(&payload[40], HostProtocol_FloatToI32(status.goal.yaw_deg * 100.0f));
  HostProtocol_WriteU32(&payload[44], elapsed_ms);
  HostProtocol_WriteU32(&payload[48], status.goal.timeout_ms);
  HostProtocol_WriteU32(&payload[52], status.updated_tick);

  HostProtocol_SendData(frame, payload, (uint8_t)sizeof(payload));
}

/* SYSTEM 命令只维护通信状态，不直接控制外设。 */
static HostProtocol_AckResult_t HostProtocol_HandleSystem(const HostProtocol_Frame_t *frame)
{
  switch (frame->cmd_id)
  {
  case 0x01U:
    /* SYS_PING：无 Payload，只用于确认协议链路和 ACK。 */
    return (frame->payload_len == 0U) ? ACK_OK : ACK_BAD_LENGTH;

  case 0x02U:
    /* SYS_HEARTBEAT：Payload 为 uint32_t 上位机时间戳，当前只用于刷新在线状态。 */
    if (frame->payload_len != 4U)
    {
      return ACK_BAD_LENGTH;
    }
    (void)HostProtocol_ReadU32(frame->payload);
    g_last_heartbeat_tick = HAL_GetTick();
    g_heartbeat_seen = 1U;
    g_heartbeat_online = 1U;
    return ACK_OK;

  case 0x03U:
    /* SYS_SET_MODE：预留控制模式，当前只保存值，不改变执行逻辑。 */
    if (frame->payload_len != 1U)
    {
      return ACK_BAD_LENGTH;
    }
    /* 系统模式不再与底盘运动控制权混用，仅保留协议兼容字段。 */
    g_control_mode = frame->payload[0];
    return ACK_OK;

  default:
    return ACK_UNKNOWN_CMD;
  }
}

/* SAFETY 命令允许打断普通底盘运动。 */
static HostProtocol_AckResult_t HostProtocol_HandleSafety(const HostProtocol_Frame_t *frame)
{
  switch (frame->cmd_id)
  {
  case 0x01U:
    /* SAFETY_ESTOP：锁定安全状态并立即停车。 */
    if (frame->payload_len != 1U)
    {
      return ACK_BAD_LENGTH;
    }
    (void)frame->payload[0];
    g_safety_latched = 1U;
    HostProtocol_StopMotion(1U);
    g_control_mode = HOST_CONTROL_SAFETY_LOCKED;
    return ACK_OK;

  case 0x02U:
    /* SAFETY_SAFE_STOP：mode=0 平滑停止，mode=1 立即停止。 */
    if (frame->payload_len != 1U)
    {
      return ACK_BAD_LENGTH;
    }
    if (frame->payload[0] == 0U)
    {
      HostProtocol_StopMotion(0U);
    }
    else if (frame->payload[0] == 1U)
    {
      HostProtocol_StopMotion(1U);
    }
    else
    {
      return ACK_BAD_PARAM;
    }
    return ACK_OK;

  case 0x03U:
    /* SAFETY_CLEAR：清除可恢复安全锁定，后续普通底盘命令可继续执行。 */
    if (frame->payload_len != 0U)
    {
      return ACK_BAD_LENGTH;
    }
    g_safety_latched = 0U;
    return ACK_OK;

  default:
    return ACK_UNKNOWN_CMD;
  }
}

/*
 * CHASSIS 命令只调用 advance_chassis 高层接口。
 *
 * 上位机不直接暴露 drive_emm / ZDT 原始帧，避免上位机绑定具体电机协议。
 */
static HostProtocol_AckResult_t HostProtocol_HandleChassis(const HostProtocol_Frame_t *frame)
{
  int16_t lf_rpm;
  int16_t rf_rpm;
  int16_t lr_rpm;
  int16_t rr_rpm;
  int16_t forward_rpm;
  int16_t strafe_rpm;
  int16_t rotate_rpm;
  int16_t vx_mm_s;
  int16_t vy_mm_s;
  int16_t wz_cdeg_s;
  int32_t target_x_mm;
  int32_t target_y_mm;
  int32_t target_yaw_cdeg;
  int16_t vmax_mm_s;
  int16_t wmax_cdeg_s;
  uint32_t timeout_ms;
  uint8_t goal_flags;
  uint8_t acc;
  AdvanceMotion_Status_t motion_status;
  WorldGoalPose2D_t goal;

  if ((HostProtocol_IsControlAllowed() == 0U) &&
      (frame->cmd_id != 0x08U) &&
      (frame->cmd_id != 0x0BU))
  {
    return ACK_DENIED;
  }

  switch (frame->cmd_id)
  {
  case 0x01U:
    /* CHASSIS_ENABLE：Payload[0] = 0 禁用，1 使能。 */
    if (frame->payload_len != 1U)
    {
      return ACK_BAD_LENGTH;
    }
    if (frame->payload[0] > 1U)
    {
      return ACK_BAD_PARAM;
    }
    if (frame->payload[0] == 0U)
    {
      HostProtocol_StopMotion(1U);
    }
    Chassis_Enable(frame->payload[0] != 0U);
    return ACK_OK;

  case 0x02U:
    /* CHASSIS_STOP：Payload[0] = 0 平滑停止，1 立即停止。 */
    if (frame->payload_len != 1U)
    {
      return ACK_BAD_LENGTH;
    }
    if (frame->payload[0] == 0U)
    {
      HostProtocol_StopMotion(0U);
    }
    else if (frame->payload[0] == 1U)
    {
      HostProtocol_StopMotion(1U);
    }
    else
    {
      return ACK_BAD_PARAM;
    }
    return ACK_OK;

  case 0x03U:
    /*
     * CHASSIS_SET_MOTOR_RPM：
     *   lf_rpm, rf_rpm, lr_rpm, rr_rpm 为 int16_t 小端，单位 RPM。
     *   acc 为 Emm 加速度参数。
     */
    if (frame->payload_len != 9U)
    {
      return ACK_BAD_LENGTH;
    }
    lf_rpm = HostProtocol_ReadI16(&frame->payload[0]);
    rf_rpm = HostProtocol_ReadI16(&frame->payload[2]);
    lr_rpm = HostProtocol_ReadI16(&frame->payload[4]);
    rr_rpm = HostProtocol_ReadI16(&frame->payload[6]);
    if ((HostProtocol_AbsRpmTooLarge(lf_rpm) != 0U) ||
        (HostProtocol_AbsRpmTooLarge(rf_rpm) != 0U) ||
        (HostProtocol_AbsRpmTooLarge(lr_rpm) != 0U) ||
        (HostProtocol_AbsRpmTooLarge(rr_rpm) != 0U))
    {
      return ACK_BAD_PARAM;
    }
    HostProtocol_SelectMotionMode(HOST_CONTROL_MOTOR_RPM);
    Chassis_SetMotorRPMEx(lf_rpm, rf_rpm, lr_rpm, rr_rpm, frame->payload[8]);
    return ACK_OK;

  case 0x04U:
    /*
     * CHASSIS_MOVE_MECANUM：
     *   forward_rpm > 0 前进
     *   strafe_rpm  > 0 右平移
     *   rotate_rpm  > 0 右旋
     */
    if (frame->payload_len != 7U)
    {
      return ACK_BAD_LENGTH;
    }
    forward_rpm = HostProtocol_ReadI16(&frame->payload[0]);
    strafe_rpm = HostProtocol_ReadI16(&frame->payload[2]);
    rotate_rpm = HostProtocol_ReadI16(&frame->payload[4]);
    if ((HostProtocol_AbsRpmTooLarge(forward_rpm) != 0U) ||
        (HostProtocol_AbsRpmTooLarge(strafe_rpm) != 0U) ||
        (HostProtocol_AbsRpmTooLarge(rotate_rpm) != 0U))
    {
      return ACK_BAD_PARAM;
    }
    HostProtocol_SelectMotionMode(HOST_CONTROL_MECANUM);
    Chassis_MoveMecanumEx(forward_rpm, strafe_rpm, rotate_rpm, frame->payload[6]);
    return ACK_OK;

  case 0x05U:
    /*
     * CHASSIS_SET_BODY_VELOCITY:
     *   vx_mm_s > 0 right, vy_mm_s > 0 forward, wz_cdeg_s > 0 counter-clockwise.
     */
    if (frame->payload_len != 7U)
    {
      return ACK_BAD_LENGTH;
    }
    vx_mm_s = HostProtocol_ReadI16(&frame->payload[0]);
    vy_mm_s = HostProtocol_ReadI16(&frame->payload[2]);
    wz_cdeg_s = HostProtocol_ReadI16(&frame->payload[4]);
    if ((HostProtocol_AbsI16Exceeds(vx_mm_s, (int32_t)CHASSIS_MAX_BODY_SPEED_MM_S) != 0U) ||
        (HostProtocol_AbsI16Exceeds(vy_mm_s, (int32_t)CHASSIS_MAX_BODY_SPEED_MM_S) != 0U) ||
        (HostProtocol_AbsI16Exceeds(wz_cdeg_s, (int32_t)(CHASSIS_MAX_BODY_WZ_DEG_S * 100.0f)) != 0U))
    {
      return ACK_BAD_PARAM;
    }
    HostProtocol_SelectMotionMode(HOST_CONTROL_BODY_VELOCITY);
    Chassis_SetBodyVelocityEx((float)vx_mm_s, (float)vy_mm_s, ((float)wz_cdeg_s) / 100.0f, frame->payload[6]);
    return ACK_OK;

  case 0x06U:
    /*
     * CHASSIS_SET_WORLD_VELOCITY:
     *   vx/vy use world axes; wz_cdeg_s > 0 counter-clockwise.
     */
    if (frame->payload_len != 7U)
    {
      return ACK_BAD_LENGTH;
    }
    vx_mm_s = HostProtocol_ReadI16(&frame->payload[0]);
    vy_mm_s = HostProtocol_ReadI16(&frame->payload[2]);
    wz_cdeg_s = HostProtocol_ReadI16(&frame->payload[4]);
    if ((HostProtocol_AbsI16Exceeds(vx_mm_s, (int32_t)CHASSIS_MAX_BODY_SPEED_MM_S) != 0U) ||
        (HostProtocol_AbsI16Exceeds(vy_mm_s, (int32_t)CHASSIS_MAX_BODY_SPEED_MM_S) != 0U) ||
        (HostProtocol_AbsI16Exceeds(wz_cdeg_s, (int32_t)(CHASSIS_MAX_BODY_WZ_DEG_S * 100.0f)) != 0U))
    {
      return ACK_BAD_PARAM;
    }
    HostProtocol_SelectMotionMode(HOST_CONTROL_WORLD_VELOCITY);
    motion_status = AdvanceMotion_SetWorldVelocityEx(
        (float)vx_mm_s,
        (float)vy_mm_s,
        ((float)wz_cdeg_s) / 100.0f,
        frame->payload[6]);
    return (motion_status == ADVANCE_MOTION_STATUS_OK) ? ACK_OK : ACK_DENIED;

  case 0x07U:
    /*
     * CHASSIS_GOTO_POSE:
     *   x_mm, y_mm, yaw_cdeg, vmax_mm_s, wmax_cdeg_s, timeout_ms, flags, acc.
     */
    if (frame->payload_len != 22U)
    {
      return ACK_BAD_LENGTH;
    }
    target_x_mm = HostProtocol_ReadI32(&frame->payload[0]);
    target_y_mm = HostProtocol_ReadI32(&frame->payload[4]);
    target_yaw_cdeg = HostProtocol_ReadI32(&frame->payload[8]);
    vmax_mm_s = HostProtocol_ReadI16(&frame->payload[12]);
    wmax_cdeg_s = HostProtocol_ReadI16(&frame->payload[14]);
    timeout_ms = HostProtocol_ReadU32(&frame->payload[16]);
    goal_flags = frame->payload[20];
    acc = frame->payload[21];

    goal.x_mm = (float)target_x_mm;
    goal.y_mm = (float)target_y_mm;
    goal.yaw_deg = ((float)target_yaw_cdeg) / 100.0f;
    goal.vmax_mm_s = (float)vmax_mm_s;
    goal.wmax_deg_s = ((float)wmax_cdeg_s) / 100.0f;
    goal.timeout_ms = timeout_ms;
    goal.goal_flags = goal_flags;
    HostProtocol_SelectMotionMode(HOST_CONTROL_GOTO_POSE);
    motion_status = AdvanceMotion_GotoPoseEx(&goal, acc);
    if (motion_status == ADVANCE_MOTION_STATUS_OK)
    {
      return ACK_OK;
    }
    return (motion_status == ADVANCE_MOTION_STATUS_INVALID_PARAM) ? ACK_BAD_PARAM : ACK_DENIED;

  case 0x08U:
    /* CHASSIS_CANCEL_GOAL: no payload. */
    if (frame->payload_len != 0U)
    {
      return ACK_BAD_LENGTH;
    }
    HostProtocol_StopMotion(0U);
    return ACK_OK;

  case 0x09U:
    /* CHASSIS_RESET_WORLD_ORIGIN: no payload. */
    if (frame->payload_len != 0U)
    {
      return ACK_BAD_LENGTH;
    }
    return (AdvanceWorld_ResetOrigin() == ADVANCE_WORLD_STATUS_OK) ? ACK_OK : ACK_DENIED;

  case 0x0BU:
    /* CHASSIS_GET_MOTION_STATUS: ACK first, then MSG_DATA with the status payload. */
    return (frame->payload_len == 0U) ? ACK_OK : ACK_BAD_LENGTH;

  default:
    return ACK_UNKNOWN_CMD;
  }
}

/* SERVO 命令调用 advance_arm，避免通信层直接操作总线舵机协议。 */
static HostProtocol_AckResult_t HostProtocol_HandleServo(const HostProtocol_Frame_t *frame)
{
  BusServo_Status_t status;

  if (HostProtocol_IsControlAllowed() == 0U)
  {
    return ACK_DENIED;
  }

  switch (frame->cmd_id)
  {
  case 0x10U:
    /*
     * ARM_GRAB：所有与实车标定相关的参数由命令携带，避免 advance_arm
     * 内部固化舵机 ID、开合位置、加速度和速度。
     */
    if (frame->payload_len != 14U)
    {
      return ACK_BAD_LENGTH;
    }
    if (frame->payload[0] > 1U)
    {
      return ACK_BAD_PARAM;
    }
    status = AdvanceArm_Grab(frame->payload[1], frame->payload[0] != 0U,
                             HostProtocol_ReadI32(&frame->payload[2]),
                             HostProtocol_ReadI32(&frame->payload[6]),
                             HostProtocol_ReadU16(&frame->payload[10]),
                             HostProtocol_ReadU16(&frame->payload[12]));
    return (status == drive_bus_servo_STATUS_OK) ? ACK_OK : ACK_FAULT;

  default:
    return ACK_UNKNOWN_CMD;
  }
}

/* 按 MsgType + CmdSet 分发命令，所有未知组合统一返回 ACK_UNKNOWN_CMD。 */
static HostProtocol_AckResult_t HostProtocol_HandleCommand(const HostProtocol_Frame_t *frame)
{
  if (frame->msg_type != MSG_CMD)
  {
    return ACK_UNKNOWN_CMD;
  }

  switch (frame->cmd_set)
  {
  case CMDSET_SYSTEM:
    return HostProtocol_HandleSystem(frame);

  case CMDSET_SAFETY:
    return HostProtocol_HandleSafety(frame);

  case CMDSET_CHASSIS:
    return HostProtocol_HandleChassis(frame);

  case CMDSET_SERVO:
    return HostProtocol_HandleServo(frame);

  default:
    return ACK_UNKNOWN_CMD;
  }
}

/*
 * 单字节喂入式解析状态机。
 *
 * 调试关注点：
 * - pos=0 等待 0x5A。
 * - pos=1 等待 0xA5，若再次收到 0x5A 会重新作为新帧头处理。
 * - 收满 9 字节基础头后检查 Version，并根据 Length 计算 expected_len。
 * - 收满 expected_len 后校验 CRC，通过才入队，失败直接丢弃。
 */
static void HostProtocol_FeedByte(HostProtocol_Source_t source, uint8_t byte)
{
  HostProtocol_Parser_t *parser = &g_comm_protocol_parser[source];
  uint16_t crc_calc;
  uint16_t crc_recv;

  if (parser->pos == 0U)
  {
    if (byte == comm_protocol_HEADER1)
    {
      parser->frame[0] = byte;
      parser->pos = 1U;
    }
    return;
  }

  if (parser->pos == 1U)
  {
    if (byte == comm_protocol_HEADER2)
    {
      parser->frame[1] = byte;
      parser->pos = 2U;
    }
    else
    {
      HostProtocol_ResetParser(parser);
      if (byte == comm_protocol_HEADER1)
      {
        parser->frame[0] = byte;
        parser->pos = 1U;
      }
    }
    return;
  }

  if (parser->pos >= comm_protocol_MAX_FRAME_LEN)
  {
    HostProtocol_ResetParser(parser);
    return;
  }

  parser->frame[parser->pos++] = byte;

  if (parser->pos == comm_protocol_HEADER_LEN)
  {
    if (parser->frame[2] != comm_protocol_VERSION)
    {
      HostProtocol_ResetParser(parser);
      return;
    }
    parser->expected_len = (uint16_t)(comm_protocol_HEADER_LEN + parser->frame[8] + comm_protocol_CRC_LEN);
  }

  if ((parser->expected_len > 0U) && (parser->pos >= parser->expected_len))
  {
    /* CRC 字段本身不参与计算；计算范围从 Version 到 Payload 结束。 */
    crc_recv = HostProtocol_ReadU16(&parser->frame[parser->expected_len - comm_protocol_CRC_LEN]);
    crc_calc = HostProtocol_Crc16Modbus(&parser->frame[2], (uint16_t)(7U + parser->frame[8]));
    if (crc_recv == crc_calc)
    {
      (void)HostProtocol_EnqueueFrame(source, parser->frame, parser->expected_len);
    }
    HostProtocol_ResetParser(parser);
  }
}

/* 注册接收源对应 UART，主要用于 ACK 回发。 */
void HostProtocol_RegisterSource(HostProtocol_Source_t source, UART_HandleTypeDef *huart)
{
  if (source >= comm_protocol_SOURCE_COUNT)
  {
    g_last_status = comm_protocol_STATUS_INVALID_PARAM;
    return;
  }

  g_comm_protocol_uart[source] = huart;
  HostProtocol_ResetParser(&g_comm_protocol_parser[source]);
  g_last_status = comm_protocol_STATUS_OK;
}

/* 从 DMA/IDLE 接收层输入原始字节，可一次输入半帧、多帧或粘包数据。 */
void HostProtocol_OnBytes(HostProtocol_Source_t source, const uint8_t *data, uint16_t length)
{
  uint16_t index;

  if ((source >= comm_protocol_SOURCE_COUNT) || (data == NULL))
  {
    g_last_status = comm_protocol_STATUS_INVALID_PARAM;
    return;
  }

  for (index = 0U; index < length; ++index)
  {
    HostProtocol_FeedByte(source, data[index]);
  }
}

/* 主循环调用：检查心跳、执行已入队命令，并给上位机回 ACK。 */
void HostProtocol_Poll(void)
{
  HostProtocol_Frame_t frame;
  HostProtocol_AckResult_t result;

  HostProtocol_RecoverTxTimeout();
  /* UART 暂忙或错误恢复后，在下一协议周期重试队首帧。 */
  HostProtocol_StartNextTx();
  HostProtocol_CheckHeartbeatTimeout();

  while (HostProtocol_DequeueFrame(&frame) != 0U)
  {
    result = HostProtocol_HandleCommand(&frame);
    HostProtocol_SendAck(&frame, result, 0U);
    if ((result == ACK_OK) &&
        (frame.msg_type == MSG_CMD) &&
        (frame.cmd_set == CMDSET_CHASSIS) &&
        (frame.cmd_id == 0x0BU))
    {
      HostProtocol_SendMotionStatusData(&frame);
    }
  }

  (void)g_control_mode;
}

void HostProtocol_OnUartTxComplete(UART_HandleTypeDef *huart)
{
  if ((g_tx_active == 0U) || (g_tx_count == 0U) ||
      (g_tx_queue[g_tx_tail].huart != huart))
  {
    return;
  }

  g_tx_tail = (uint8_t)((g_tx_tail + 1U) % comm_protocol_TX_QUEUE_SIZE);
  --g_tx_count;
  g_tx_active = 0U;
  g_tx_started_tick = 0U;
  HostProtocol_StartNextTx();
}

void HostProtocol_OnUartError(UART_HandleTypeDef *huart)
{
  if ((g_tx_active != 0U) && (g_tx_count != 0U) &&
      (g_tx_queue[g_tx_tail].huart == huart))
  {
    g_tx_tail = (uint8_t)((g_tx_tail + 1U) % comm_protocol_TX_QUEUE_SIZE);
    --g_tx_count;
    g_tx_active = 0U;
    g_tx_started_tick = 0U;
    g_last_status = comm_protocol_STATUS_OVERFLOW;
    HostProtocol_StartNextTx();
  }
}

HostProtocol_Status_t HostProtocol_GetLastStatus(void)
{
  return g_last_status;
}

uint32_t HostProtocol_GetTxTimeoutCount(void)
{
  return g_tx_timeout_count;
}
