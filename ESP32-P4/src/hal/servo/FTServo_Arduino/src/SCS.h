/*
 * SCS.h
 * 飞特串行舵机通信层协议程序
 * 日期: 2024.11.21
 * 作者: txl
 */

#ifndef _SCS_H
#define _SCS_H

#include "INST.h"

// Local read evidence only; never added to the host wire protocol.
struct SCSReadDiagnostics {
	u8 payloadBytes = 0;   // Actual data bytes copied, including a partial reply.
	u8 replyLength = 0;    // Raw LENGTH byte, zero if no complete reply header.
	bool statusValid = false; // Status belongs to a complete checksum-valid reply.
};

class SCS{
public:
	/** 当前产品单 BUS 最大同步回包为 12 * (15 + 6) = 252 byte。 */
	static constexpr u16 kSyncReadRxBufferCapacity = 256;
	SCS();
	SCS(u8 End);
	SCS(u8 End, u8 Level);
	int genWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);//普通写指令
	int regWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);//异步写指令
	int RegWriteAction(u8 ID = 0xfe);//异步写执行指令
	void syncWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen);//同步写指令
	int writeByte(u8 ID, u8 MemAddr, u8 bDat);//写1个字节
	int writeWord(u8 ID, u8 MemAddr, u16 wDat);//写2个字节
	int Read(u8 ID, u8 MemAddr, u8 *nData, u8 nLen);//读指令
	int readByte(u8 ID, u8 MemAddr);//读1个字节
	int readWord(u8 ID, u8 MemAddr);//读2个字节
	int Ping(u8 ID);//Ping指令
	int syncReadPacketTx(u8 ID[], u8 IDN, u8 MemAddr, u8 nLen);//同步读指令包发送
	int syncReadPacketTxOnly(u8 ID[], u8 IDN, u8 MemAddr, u8 nLen);//仅发送同步读请求，不等待回包
	int syncReadCollectAvailable();//非阻塞搬运当前UART已收到的同步读字节
	int syncReadPacketRx(u8 ID, u8 *nDat);//同步读返回包解码，成功返回内存字节数，失败返回0
	int syncReadRxPacketToByte();//解码一个字节
	int syncReadRxPacketToWrod(u8 negBit=0);//解码两个字节，negBit为方向为，negBit=0表示无方向
	void syncReadBegin(u8 IDN, u8 rxLen, u32 TimeOut);//同步读开始
	void syncReadEnd();//同步读结束
	u16 syncReadBufferedBytes() const { return syncReadRxBuffLen; }
	u16 syncReadExpectedBytes() const { return syncReadRxBuffMax; }
	int Reset(u8 ID);//重置舵机状态
	int Recal(u8 ID);//重置舵机中位
	// STS firmware >= 3.10: opcode 0x0B with an encoded little-endian position,
	// no register address. Unicast requires a valid zero-status ACK.
	int Recal(u8 ID, u16 EncodedPosition);
	u8 getState() { return u8Status; }
	u8 getLastError() { return u8Error; }
	const SCSReadDiagnostics& getReadDiagnostics() const { return readDiagnostics; }
public:
	u8 Level;//舵机返回等级
	u8 End;//处理器大小端结构
	u8 u8Status;//舵机状态
	u8 u8Error;//通信状态
	u8 syncReadRxPacketIndex = 0;
	u8 syncReadRxPacketLen = 0;
	u8 *syncReadRxPacket = NULL;
	/** 固定缓冲替代每次同步读的 new/delete，避免高频堆碎片和分配抖动。 */
	u8 syncReadRxBuff[kSyncReadRxBufferCapacity] = {0};
	u16 syncReadRxBuffLen = 0;
	u16 syncReadRxBuffMax = 0;
	u32 syncTimeOut = 0;
protected:
	SCSReadDiagnostics readDiagnostics;
	virtual int writeSCS(unsigned char *nDat, int nLen) = 0;
	virtual int readSCS(unsigned char *nDat, int nLen) = 0;
	virtual int readSCS(unsigned char *nDat, int nLen, unsigned long TimeOut) = 0;
	virtual int readSCSAvailable(unsigned char *nDat, int nLen) = 0;
	virtual int writeSCS(unsigned char bDat) = 0;
	virtual void rFlushSCS() = 0;
	virtual void wFlushSCS() = 0;
protected:
	void writeBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun);
	void Host2SCS(u8 *DataL, u8* DataH, u16 Data);//1个16位数拆分为2个8位数
	u16	SCS2Host(u8 DataL, u8 DataH);//2个8位数组合为1个16位数
	int	Ack(u8 ID);//返回应答
	int checkHead();//帧头检测
};
#endif
