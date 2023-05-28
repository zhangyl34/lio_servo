#include "can_adapter.h"
#include "controlcan.h"
#include "stdio.h"
#include "string.h"
#include <QMap>

QMap<unsigned short, Canopen*> map_canopen;

CanDeviceComm::CanDeviceComm() {
	device_type = VCI_USBCAN2;
	device_index = 0;
	can_index = 0;
}

CanDeviceComm::~CanDeviceComm() {

}

bool CanDeviceComm::open() {
    // 打开设备
	if(VCI_OpenDevice(device_type, device_index, 0) == 1) {
		printf("open deivce success!\n");  // 打开设备成功

        // 初始化参数，严格参数二次开发函数库说明书。
        VCI_INIT_CONFIG config;
        config.AccCode=0;
        config.AccMask=0xFFFFFFFF;
        config.Filter=1;      // 接收所有帧
        config.Timing0=0x00;  // 1M
        config.Timing1=0x14;
        config.Mode=0;        // 正常模式		
        
        if(VCI_InitCAN(device_type, device_index, can_index, &config)!=1)
        {
            printf("init CAN1 error\n");
            VCI_CloseDevice(device_type,0);
            return false;
        }

        if(VCI_StartCAN(device_type, device_index, can_index)!=1)
        {
            printf("start CAN1 error\n");
            VCI_CloseDevice(device_type,0);
            return false;
        }

        int send_timeout = 1;
		VCI_SetReference(device_type, device_index, can_index, 4, &send_timeout);  // 设置发送超时
        start();
        return true;
    } else {
		printf("open deivce error!\n");
		return false;
	}
}

bool CanDeviceComm::close() {
    VCI_CloseDevice(device_type, device_index);
}

bool CanDeviceComm::sendData(CanMessageStr& message_data) {
    send_queue.enqueue(message_data);
    return true;
}

bool CanDeviceComm::recvData(CanMessageStr& recv_msg) {
    VCI_CAN_OBJ canalyst_ii_msg;
    if (VCI_Receive(device_type, device_index, can_index, &canalyst_ii_msg, 1, 0) <= 0) {
        return false;
    } else {
        recv_msg.id = canalyst_ii_msg.ID;
        recv_msg.data_len = canalyst_ii_msg.DataLen;
        recv_msg.is_extern_flag = canalyst_ii_msg.ExternFlag;
        recv_msg.is_remote_flag = canalyst_ii_msg.RemoteFlag;
        memcpy((void*)recv_msg.data, (void*)canalyst_ii_msg.Data, canalyst_ii_msg.DataLen);
        return true;
    }
}

void CanDeviceComm::run() {
    CanMessageStr recv_msg;
    CanMessageStr send_msg;
    while(true) {
        if (recvData(recv_msg) == true) {
            switch(recv_msg.id & 0x780U) {
                case 0x80U: {
                    map_canopen[recv_msg.id & 0x7F]->recvEmergency(recv_msg.data);
                    break;
                }
                case 0x180U: {
                    map_canopen[recv_msg.id & 0x7F]->recvTpdo(recv_msg.id & 0x780U, recv_msg.data);
                    break;
                }
                case 0x280U: {
                    map_canopen[recv_msg.id & 0x7F]->recvTpdo(recv_msg.id & 0x780U, recv_msg.data);
                    break;
                }
                case 0x380U: {
                    map_canopen[recv_msg.id & 0x7F]->recvTpdo(recv_msg.id & 0x780U, recv_msg.data);
                    break;
                }
                case 0x480U: {
                    map_canopen[recv_msg.id & 0x7F]->recvTpdo(recv_msg.id & 0x780U, recv_msg.data);
                }
                case 0x580U: {
                    printf("recv data:0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x!\n", recv_msg.id, recv_msg.data[0], 
                        recv_msg.data[1], recv_msg.data[2], recv_msg.data[3], recv_msg.data[4], recv_msg.data[5]);
                    map_canopen[recv_msg.id & 0x7F]->recvSdo(recv_msg.data);
                    break;
                }
                case 0x700U: {
                    if (recv_msg.data_len == 1) {
                        if (recv_msg.data[0] == 0) {
                            map_canopen[recv_msg.id & 0x7F]->recvBootup();
                        } else {

                        }
                    } else {

                    }
                    break;
                }
                default: {
                    break;
                }
            }
        }
        if (send_queue.isEmpty() != true) {
            send_msg = send_queue.dequeue();
            VCI_CAN_OBJ canalyst_ii_obj;
            canalyst_ii_obj.ID = send_msg.id;
            canalyst_ii_obj.SendType = 1;
            canalyst_ii_obj.DataLen = send_msg.data_len;
            canalyst_ii_obj.ExternFlag = send_msg.is_extern_flag;
            canalyst_ii_obj.RemoteFlag = send_msg.is_remote_flag;
            memcpy((void*)canalyst_ii_obj.Data, (void*)send_msg.data, send_msg.data_len);
            printf("send data:0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x!\n", send_msg.id, send_msg.data[0], 
                send_msg.data[1], send_msg.data[2], send_msg.data[3], send_msg.data[4], send_msg.data[5]);
            if (1 == VCI_Transmit(device_type, device_index, can_index, &canalyst_ii_obj, 1)) {
         
            } else {
                printf("send data err\n");
            }
        }
        usleep(200);
    }
}

CanDeviceComm *Canopen::comm = nullptr;
Canopen::Canopen(unsigned short node_id) : node_id(node_id) {
    if (comm == nullptr) {
        comm = new CanDeviceComm();
        comm->open();
    }
    map_canopen.insert(node_id, this);
}
    
Canopen::~Canopen() {

}

void Canopen::recvEmergency(unsigned char *const) {

}

void Canopen::recvTpdo(unsigned short, unsigned char *const) {

}

void Canopen::recvBootup() {

}

bool Canopen::writeOD(unsigned char *data, unsigned char len, unsigned short index, unsigned char sub_index) {
    CanMessageStr message_data;
    message_data.id = 0x600 + node_id;
    message_data.data_len = 8;
    message_data.is_extern_flag = 0;
    message_data.is_remote_flag = 0;
    memset(message_data.data, 0, 8);
    if (len == 1) {
        message_data.data[0] = 0x2FU;
        memcpy(&message_data.data[4], data, 1);
    } else if (len == 2) {
        message_data.data[0] = 0x2BU;
        memcpy(&message_data.data[4], data, 2);
    } else if (len == 3) {
        message_data.data[0] = 0x27U;
        memcpy(&message_data.data[4], data, 3);
    } else if (len == 4) {
        message_data.data[0] = 0x23U;
        memcpy(&message_data.data[4], data, 4);
    } else {
        printf("write od len err:%d!\n", len);
        return false;
    }

    message_data.data[1] = index & 0xFFU;
    message_data.data[2] = index >> 8;
    message_data.data[3] = sub_index;

    memcpy(send_sdo_data, message_data.data, 8);
    memset(recv_sdo_data, 0, 8);
    comm->sendData(message_data);

    // if (true == sdo_handle_sem.tryAcquire(1, 500)) {
    //     if (recv_sdo_data[0] == 0x60U) {
    //         return true;
    //     } else {
    //         printf("write od err:cmd=0x%x!\n", recv_sdo_data[0]);
    //         return false;
    //     }
    // } else {
    //     printf("write od timeout!\n");
    //     return false;
    // }
}

bool Canopen::readOD(unsigned char *data, unsigned char len, unsigned short index, unsigned char sub_index) {
    CanMessageStr message_data;
    message_data.id = 0x600 + node_id;
    message_data.data_len = 8;
    message_data.is_extern_flag = 0;
    message_data.is_remote_flag = 0;
    memset(message_data.data, 0, 8);
    message_data.data[0] = 0x40U;
    message_data.data[1] = index & 0xFFU;
    message_data.data[2] = index >> 8;
    message_data.data[3] = sub_index;

    memcpy(send_sdo_data, message_data.data, 8);
    memset(recv_sdo_data, 0, 8);
    comm->sendData(message_data);

    // if (true == sdo_handle_sem.tryAcquire(1, 500)) {
    //     if ((recv_sdo_data[0] == 0x4FU) && (1 == len)) {
    //         memcpy(data, recv_sdo_data, len);
    //         return true;
    //     } else if ((recv_sdo_data[0] == 0x4BU) && (2 == len)) {
    //         memcpy(data, recv_sdo_data, len);
    //         return true;
    //     } else if ((recv_sdo_data[0] == 0x47U) && (3 == len)) {
    //         memcpy(data, recv_sdo_data, len);
    //         return true;
    //     } else if ((recv_sdo_data[0] == 0x43U) && (4 == len)) {
    //         memcpy(data, recv_sdo_data, len);
    //         return true;
    //     } else {
    //         printf("sdo val err:expect len=%d, actual cmd=0x%x!\n", len, recv_sdo_data[0]);
    //         return false;
    //     }
    // } else {
    //     printf("read od timeout!\n");
    //     return false;
    // }
}

bool Canopen::sendRpdo(unsigned short pdo_id, unsigned char *const data, unsigned char len) {
    CanMessageStr message_data;
    message_data.id = node_id + pdo_id;
    message_data.data_len = len;
    message_data.is_extern_flag = 0;
    message_data.is_remote_flag = 0;
    if (len > 8) {
        return false;
    }
    memcpy(message_data.data, data, len);
    comm->sendData(message_data);
    return true;
}

bool Canopen::setNodeRun() {
    CanMessageStr message_data;
    message_data.id = 0U;
    message_data.data_len = 2;
    message_data.is_extern_flag = 0;
    message_data.is_remote_flag = 0;
    message_data.data[0] = 0x1;
    message_data.data[1] = node_id;

    comm->sendData(message_data);
    return true;
}

void Canopen::resetNode() {
    CanMessageStr message_data;
    message_data.id = 0U;
    message_data.data_len = 2;
    message_data.is_extern_flag = 0;
    message_data.is_remote_flag = 0;
    message_data.data[0] = 0x81;
    message_data.data[1] = node_id;

    comm->sendData(message_data);
}

void Canopen::recvSdo(unsigned char *data) {
    if ((send_sdo_data[1] == data[1]) && 
        (send_sdo_data[2] == data[2]) &&
        (send_sdo_data[3] == data[3])) {
            memcpy(recv_sdo_data, data, 8);
            //sdo_handle_sem.release(1);
    }
}
