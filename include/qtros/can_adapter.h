#pragma once

#include "QThread"
#include "QSemaphore"
#include "QQueue"

typedef struct {
	unsigned int id;
	unsigned char is_remote_flag;
	unsigned char is_extern_flag;
	unsigned char data_len;
	unsigned char data[8];
} CanMessageStr;

class CanDeviceComm : public QThread {
Q_OBJECT

public:
    CanDeviceComm();
    ~CanDeviceComm();
    bool open();
    bool close();
    bool sendData(CanMessageStr&);
    bool recvData(CanMessageStr&);

private:
    void run();

    int device_type;
    int device_index;
    int can_index;
    QQueue<CanMessageStr> send_queue;
};

class Canopen {
public:
    Canopen(unsigned short node_id);
    ~Canopen();
    virtual void recvEmergency(unsigned char *const);
    virtual void recvTpdo(unsigned short, unsigned char *const);
    virtual void recvBootup();
    bool writeOD(unsigned char *data, unsigned char len, unsigned short index, unsigned char sub_index);
    bool readOD(unsigned char *data, unsigned char len, unsigned short index, unsigned char sub_index);
    bool sendRpdo(unsigned short, unsigned char *const, unsigned char);
    bool setNodeRun();
    void resetNode();
    void recvSdo(unsigned char *data);
    static CanDeviceComm *comm;
    unsigned char recv_sdo_data[8];
    unsigned char send_sdo_data[8];
    QSemaphore sdo_handle_sem;
private:
    unsigned short node_id;
};


