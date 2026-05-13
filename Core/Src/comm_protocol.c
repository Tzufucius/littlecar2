#include "comm_protocol.h"
#include "advance_chassis.h"
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
#define comm_protocol_HEADER1          ((uint8_t)0x5AU)
#define comm_protocol_HEADER2          ((uint8_t)0xA5U)
#define comm_protocol_VERSION          ((uint8_t)0x01U)
#define comm_protocol_HEADER_LEN       ((uint16_t)9U)
#define comm_protocol_CRC_LEN          ((uint16_t)2U)
#define comm_protocol_MAX_PAYLOAD      ((uint16_t)255U)
#define comm_protocol_MAX_FRAME_LEN    (comm_protocol_HEADER_LEN + comm_protocol_MAX_PAYLOAD + comm_protocol_CRC_LEN)
#define comm_protocol_QUEUE_SIZE       ((uint8_t)4U)
#define comm_protocol_TX_TIMEOUT_MS    ((uint32_t)20U)
#define comm_protocol_HEARTBEAT_MS     ((uint32_t)300U)

typedef enum
{
  /* 上位机发给 STM32 的控制命令。 */
  MSG_CMD = 0x01,
  /* STM32 对命令接收和执行结果的确认。 */
  MSG_ACK = 0x02
} HostProtocol_MsgType_t;

typedef enum
{
  /* 通信测试、心跳、模式设置。 */
  CMDSET_SYSTEM = 0x01,
  /* 急停、安全停止、安全状态清除。 */
  CMDSET_SAFETY = 0x02,
  /* 底盘使能、停止、四轮 RPM、麦克纳姆合成速度。 */
  CMDSET_CHASSIS = 0x03
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

static void HostProtocol_WriteU16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8) & 0xFFU);
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
    Chassis_Stop();
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

  (void)HAL_UART_Transmit(huart, tx, (uint16_t)sizeof(tx), comm_protocol_TX_TIMEOUT_MS);
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
      Chassis_Stop();
      return ACK_OK;

    case 0x02U:
      /* SAFETY_SAFE_STOP：mode=0 平滑停止，mode=1 立即停止。 */
      if (frame->payload_len != 1U)
      {
        return ACK_BAD_LENGTH;
      }
      if (frame->payload[0] == 0U)
      {
        Chassis_SmoothStop(CHASSIS_DEFAULT_ACC);
      }
      else if (frame->payload[0] == 1U)
      {
        Chassis_Stop();
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

  if (HostProtocol_IsControlAllowed() == 0U)
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
        Chassis_SmoothStop(CHASSIS_DEFAULT_ACC);
      }
      else if (frame->payload[0] == 1U)
      {
        Chassis_Stop();
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
      Chassis_MoveMecanumEx(forward_rpm, strafe_rpm, rotate_rpm, frame->payload[6]);
      return ACK_OK;

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

  HostProtocol_CheckHeartbeatTimeout();

  while (HostProtocol_DequeueFrame(&frame) != 0U)
  {
    result = HostProtocol_HandleCommand(&frame);
    HostProtocol_SendAck(&frame, result, 0U);
  }

  (void)g_control_mode;
}

HostProtocol_Status_t HostProtocol_GetLastStatus(void)
{
  return g_last_status;
}
