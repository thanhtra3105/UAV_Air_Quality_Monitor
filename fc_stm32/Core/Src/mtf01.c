/*
 * mtf01.c
 */

#include "mtf01.h"
#include <stdlib.h>
#include <string.h>

uint8_t uart_rx_mtf01;
MICOLINK_MSG_t micolink_msg = {0};
volatile uint8_t mtf01_data_ready = 0;

#define FLOW_DEADBAND 2

// ---------------------------------------------------------
// CÁC HÀM PARSER CHUẨN TỪ TÀI LIỆU MICOAIR
// ---------------------------------------------------------
bool micolink_check_sum(MICOLINK_MSG_t* msg) {
    uint8_t length = msg->len + 6;
    uint8_t temp[MICOLINK_MAX_LEN];
    uint8_t checksum = 0;

    memcpy(temp, msg, length);

    for(uint8_t i = 0; i < length; i++) {
        checksum += temp[i];
    }

    if(checksum == msg->checksum) return true;
    else return false;
}

bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data) {
    switch(msg->status) {
        case 0:     // Header
            if(data == MICOLINK_MSG_HEAD) {
                msg->head = data;
                msg->status++;
            }
            break;
        case 1:     // Device ID
            msg->dev_id = data;
            msg->status++;
            break;
        case 2:     // System ID
            msg->sys_id = data;
            msg->status++;
            break;
        case 3:     // Message ID
            msg->msg_id = data;
            msg->status++;
            break;
        case 4:     // Sequence
            msg->seq = data;
            msg->status++;
            break;
        case 5:     // Payload Length
            msg->len = data;
            if(msg->len == 0) msg->status += 2;
            else if(msg->len > MICOLINK_MAX_PAYLOAD_LEN) msg->status = 0;
            else msg->status++;
            break;
        case 6:     // Payload Data
            msg->payload[msg->payload_cnt++] = data;
            if(msg->payload_cnt == msg->len) {
                msg->payload_cnt = 0;
                msg->status++;
            }
            break;
        case 7:     // Checksum
            msg->checksum = data;
            msg->status = 0;
            if(micolink_check_sum(msg)) return true;
            break;
        default:
            msg->status = 0;
            msg->payload_cnt = 0;
            break;
    }
    return false;
}

// ---------------------------------------------------------
// GIAO TIẾP VỚI STM32 HAL & XỬ LÝ DỮ LIỆU
// ---------------------------------------------------------

void MTF01_Init(UART_HandleTypeDef *huart) {
    memset(&micolink_msg, 0, sizeof(MICOLINK_MSG_t));
    HAL_UART_Receive_IT(huart, &uart_rx_mtf01, 1);
}

// Hàm này nhét vào HAL_UART_RxCpltCallback
void MTF01_Process(UART_HandleTypeDef *huart) {
    if (!mtf01_data_ready) {
        if (micolink_parse_char(&micolink_msg, uart_rx_mtf01)) {
            mtf01_data_ready = 1;
        }
    }
    HAL_UART_Receive_IT(huart, &uart_rx_mtf01, 1);
}

// Hàm này gọi trong vòng lặp while(1) hoặc task của FreeRTOS
uint8_t MTF01_Update(MTF01_t *mtf) {
    if (mtf01_data_ready) {
        // Chỉ xử lý nếu đúng ID của cảm biến
        if (micolink_msg.msg_id == MICOLINK_MSG_ID_RANGE_SENSOR) {
            MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
            memcpy(&payload, micolink_msg.payload, micolink_msg.len);

            mtf->distance = payload.distance;
            mtf->dist_quality = payload.strength;
            mtf->flow_x = payload.flow_vel_x;
            mtf->flow_y = payload.flow_vel_y;
            mtf->flow_quality = payload.flow_quality;

            // Lọc nhiễu Deadband
            if (abs(mtf->flow_x) <= FLOW_DEADBAND) mtf->flow_x = 0;
            if (abs(mtf->flow_y) <= FLOW_DEADBAND) mtf->flow_y = 0;

            // Chỉ tính toán nếu Lidar đọc > 10mm và tín hiệu Flow ổn định
            if (mtf->flow_quality > 20 && mtf->distance >= 10) {
                float h_m = (float)mtf->distance * 0.001f;

                // Công thức hãng: V(cm/s) = Flow * H(m). Từ đó chia 100 để ra V(m/s)
                mtf->vx = (mtf->flow_x * h_m) / 100.0f;
                mtf->vy = (mtf->flow_y * h_m) / 100.0f;
            } else {
                mtf->vx = 0;
                mtf->vy = 0;
            }
        }

        mtf01_data_ready = 0; // Giải phóng cờ để đọc gói mới
        return 1;
    }
    return 0;
}




//
//
///*
// * mtf01.c
// *
// * Part 2
// */
//
//#include "mtf01.h"
//#include <string.h>
//
///*==========================================================
// *
// * Local Function
// *
// *==========================================================*/
//
//static bool Micolink_Checksum(MICOLINK_Frame_t *frame)
//{
//    uint8_t checksum = 0;
//
//    checksum += frame->head;
//    checksum += frame->dev_id;
//    checksum += frame->sys_id;
//    checksum += frame->msg_id;
//    checksum += frame->seq;
//    checksum += frame->len;
//
//    for(uint8_t i=0;i<frame->len;i++)
//    {
//        checksum += frame->payload[i];
//    }
//
//    return (checksum == frame->checksum);
//}
//
///*==========================================================
// *
// * Initialize
// *
// *==========================================================*/
//
//void MTF01_Init(MTF01_Handle_t *mtf,
//                UART_HandleTypeDef *huart)
//{
//    memset(mtf,0,sizeof(MTF01_Handle_t));
//
//    mtf->huart = huart;
//
//    mtf->frame.state = MICOLINK_WAIT_HEAD;
//
//    HAL_UART_Receive_IT(mtf->huart,
//                        &mtf->rx_byte,
//                        1);
//}
//
///*==========================================================
// *
// * Parser
// *
// *==========================================================*/
//
//void MTF01_InputByte(MTF01_Handle_t *mtf,
//                     uint8_t byte)
//{
//    MICOLINK_Frame_t *frame = &mtf->frame;
//
//    switch(frame->state)
//    {
//
//    /*------------------------------------------*/
//
//    case MICOLINK_WAIT_HEAD:
//
//        if(byte == MICOLINK_HEAD)
//        {
//            frame->head = byte;
//            frame->payload_index = 0;
//            frame->state = MICOLINK_DEV_ID;
//        }
//
//        break;
//
//    /*------------------------------------------*/
//
//    case MICOLINK_DEV_ID:
//
//        frame->dev_id = byte;
//        frame->state = MICOLINK_SYS_ID;
//
//        break;
//
//    /*------------------------------------------*/
//
//    case MICOLINK_SYS_ID:
//
//        frame->sys_id = byte;
//        frame->state = MICOLINK_MSG_ID;
//
//        break;
//
//    /*------------------------------------------*/
//
//    case MICOLINK_MSG_ID:
//
//        frame->msg_id = byte;
//        frame->state = MICOLINK_SEQ;
//
//        break;
//
//    /*------------------------------------------*/
//
//    case MICOLINK_SEQ:
//
//        frame->seq = byte;
//        frame->state = MICOLINK_LEN;
//
//        break;
//
//    /*------------------------------------------*/
//
//    case MICOLINK_LEN:
//
//        frame->len = byte;
//
//        if(frame->len > MICOLINK_MAX_PAYLOAD)
//        {
//            frame->state = MICOLINK_WAIT_HEAD;
//        }
//        else if(frame->len == 0)
//        {
//            frame->state = MICOLINK_CHECKSUM;
//        }
//        else
//        {
//            frame->payload_index = 0;
//            frame->state = MICOLINK_PAYLOAD;
//        }
//
//        break;
//
//    /*------------------------------------------*/
//
//    case MICOLINK_PAYLOAD:
//
//        frame->payload[frame->payload_index++] = byte;
//
//        if(frame->payload_index >= frame->len)
//        {
//            frame->state = MICOLINK_CHECKSUM;
//        }
//
//        break;
//
//    /*------------------------------------------*/
//
//    case MICOLINK_CHECKSUM:
//
//        frame->checksum = byte;
//
//        frame->state = MICOLINK_WAIT_HEAD;
//
//        if(Micolink_Checksum(frame))
//        {
//            mtf->new_data = 1;
//        }
//
//        break;
//
//    /*------------------------------------------*/
//
//    default:
//
//        frame->state = MICOLINK_WAIT_HEAD;
//        frame->payload_index = 0;
//
//        break;
//
//    }
//
//}
//
///*==========================================================
// *
// * UART Callback
// *
// *==========================================================*/
//
//void MTF01_UARTCallback(MTF01_Handle_t *mtf)
//{
//    MTF01_InputByte(mtf,
//                    mtf->rx_byte);
//
//    HAL_UART_Receive_IT(mtf->huart,
//                        &mtf->rx_byte,
//                        1);
//}
//
//
//
//
///*==========================================================
// *
// * Read Data
// *
// *==========================================================*/
//
//bool MTF01_Read(MTF01_Handle_t *mtf,
//                MTF01_Data_t *data)
//{
//    const MICOLINK_RangePayload_t *payload;
//
//    if(mtf->new_data == 0)
//    {
//        return false;
//    }
//
//    mtf->new_data = 0;
//
//    /*------------------------------------------------------
//     * Check Message ID
//     *-----------------------------------------------------*/
//
//    if(mtf->frame.msg_id != MICOLINK_MSG_RANGE_SENSOR)
//    {
//        return false;
//    }
//
//    /*------------------------------------------------------
//     * Check Payload Length
//     *-----------------------------------------------------*/
//
//    if(mtf->frame.len != sizeof(MICOLINK_RangePayload_t))
//    {
//        return false;
//    }
//
//    /*------------------------------------------------------
//     * Decode Payload
//     *-----------------------------------------------------*/
//
//    payload = (const MICOLINK_RangePayload_t *)mtf->frame.payload;
//
//    mtf->data.timestamp_ms = payload->time_ms;
//
//    mtf->data.distance_mm = payload->distance;
//
//    mtf->data.strength = payload->strength;
//
//    mtf->data.precision = payload->precision;
//
//    mtf->data.tof_status = payload->tof_status;
//
//    mtf->data.flow_vel_x = payload->flow_vel_x;
//
//    mtf->data.flow_vel_y = payload->flow_vel_y;
//
//    mtf->data.flow_quality = payload->flow_quality;
//
//    mtf->data.flow_status = payload->flow_status;
//
//    *data = mtf->data;
//
//    return true;
//}
