/*
 * SCS.cpp
 * 飞特串行舵机通信层协议程序
 * 日期: 2024.12.21
 * 作者: txl
 */

#include <stddef.h>
#include "SCS.h"

SCS::SCS()
{
	Level = 1;//除广播指令所有指令返回应答
	u8Status = 0;
}

SCS::SCS(u8 End)
{
	Level = 1;
	this->End = End;
	u8Status = 0;
}

SCS::SCS(u8 End, u8 Level)
{
	this->Level = Level;
	this->End = End;
	u8Status = 0;
}

//1个16位数拆分为2个8位数
//DataL为低位，DataH为高位
void SCS::Host2SCS(u8 *DataL, u8* DataH, u16 Data)
{
	if(End){
		*DataL = (Data>>8);
		*DataH = (Data&0xff);
	}else{
		*DataH = (Data>>8);
		*DataL = (Data&0xff);
	}
}

//2个8位数组合为1个16位数
//DataL为低位，DataH为高位
u16 SCS::SCS2Host(u8 DataL, u8 DataH)
{
	u16 Data;
	if(End){
		Data = DataL;
		Data<<=8;
		Data |= DataH;
	}else{
		Data = DataH;
		Data<<=8;
		Data |= DataL;
	}
	return Data;
}

void SCS::writeBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun)
{
	u8 msgLen = 2;
	u8 bBuf[6];
	u8 CheckSum = 0;
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = ID;
	bBuf[4] = Fun;
	if(nDat){
		msgLen += nLen + 1;
		bBuf[3] = msgLen;
		bBuf[5] = MemAddr;
		writeSCS(bBuf, 6);
		
	}else{
		bBuf[3] = msgLen;
		writeSCS(bBuf, 5);
	}
	CheckSum = ID + msgLen + Fun + MemAddr;
	u8 i = 0;
	if(nDat){
		for(i=0; i<nLen; i++){
			CheckSum += nDat[i];
		}
		writeSCS(nDat, nLen);
	}
	writeSCS(~CheckSum);
}

//普通写指令
//舵机ID，MemAddr内存表地址，写入数据，写入长度
int SCS::genWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen)
{
	rFlushSCS();
	writeBuf(ID, MemAddr, nDat, nLen, INST_WRITE);
	wFlushSCS();
	return Ack(ID);
}

//异步写指令
//舵机ID，MemAddr内存表地址，写入数据，写入长度
int SCS::regWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen)
{
	rFlushSCS();
	writeBuf(ID, MemAddr, nDat, nLen, INST_REG_WRITE);
	wFlushSCS();
	return Ack(ID);
}

//异步写执行指令
//舵机ID
int SCS::RegWriteAction(u8 ID)
{
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, INST_REG_ACTION);
	wFlushSCS();
	return Ack(ID);
}

//同步写指令
//舵机ID[]数组，IDN数组长度，MemAddr内存表地址，写入数据，写入长度
void SCS::syncWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen)
{
	rFlushSCS();
	u8 mesLen = ((nLen+1)*IDN+4);
	u8 Sum = 0;
	u8 bBuf[7];
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = 0xfe;
	bBuf[3] = mesLen;
	bBuf[4] = INST_SYNC_WRITE;
	bBuf[5] = MemAddr;
	bBuf[6] = nLen;
	writeSCS(bBuf, 7);

	Sum = 0xfe + mesLen + INST_SYNC_WRITE + MemAddr + nLen;
	u8 i, j;
	for(i=0; i<IDN; i++){
		writeSCS(ID[i]);
		writeSCS(nDat+i*nLen, nLen);
		Sum += ID[i];
		for(j=0; j<nLen; j++){
			Sum += nDat[i*nLen+j];
		}
	}
	writeSCS(~Sum);
	wFlushSCS();
}

int SCS::writeByte(u8 ID, u8 MemAddr, u8 bDat)
{
	rFlushSCS();
	writeBuf(ID, MemAddr, &bDat, 1, INST_WRITE);
	wFlushSCS();
	return Ack(ID);
}

int SCS::writeWord(u8 ID, u8 MemAddr, u16 wDat)
{
	u8 bBuf[2];
	Host2SCS(bBuf+0, bBuf+1, wDat);
	rFlushSCS();
	writeBuf(ID, MemAddr, bBuf, 2, INST_WRITE);
	wFlushSCS();
	return Ack(ID);
}

//读指令
//舵机ID，MemAddr内存表地址，返回数据nData，数据长度nLen
int SCS::Read(u8 ID, u8 MemAddr, u8 *nData, u8 nLen)
{
	readDiagnostics = {};
	u8Status = 0;
	rFlushSCS();
	writeBuf(ID, MemAddr, &nLen, 1, INST_READ);
	wFlushSCS();
	u8Error = 0;
	if(!checkHead()){
		u8Error = ERR_NO_REPLY;
		return 0;
	}
	u8 bBuf[4];
	u8Status = 0;
	if(readSCS(bBuf, 3)!=3){
		u8Error = ERR_NO_REPLY;
		return 0;
	}
	readDiagnostics.replyLength = bBuf[1];
	u8Status = bBuf[2];
	if(bBuf[0]!=ID && ID!=0xfe){
		u8Error = ERR_SLAVE_ID;
		return 0;
	}
	if(bBuf[1]!=(nLen+2)){
		u8Error = ERR_BUFF_LEN;
		return 0;
	}
	int Size = readSCS(nData, nLen);
	readDiagnostics.payloadBytes = Size;
	if(Size!=nLen){
		u8Error = ERR_NO_REPLY;
		return 0;
	}
	if(readSCS(bBuf+3, 1)!=1){
		u8Error = ERR_NO_REPLY;
		return 0;
	}
	u8 calSum = bBuf[0]+bBuf[1]+bBuf[2];
	u8 i;
	for(i=0; i<Size; i++){
		calSum += nData[i];
	}
	calSum = ~calSum;
	if(calSum!=bBuf[3]){
		u8Error = ERR_CRC_CMP;
		return 0;
	}
	u8Status = bBuf[2];
	readDiagnostics.statusValid = true;
	return Size;
}

//读1字节，超时返回-1
int SCS::readByte(u8 ID, u8 MemAddr)
{
	u8 bDat;
	int Size = Read(ID, MemAddr, &bDat, 1);
	if(Size!=1){
		return -1;
	}else{
		return bDat;
	}
}

//读2字节，超时返回-1
int SCS::readWord(u8 ID, u8 MemAddr)
{	
	u8 nDat[2];
	int Size;
	u16 wDat;
	Size = Read(ID, MemAddr, nDat, 2);
	if(Size!=2)
		return -1;
	wDat = SCS2Host(nDat[0], nDat[1]);
	return wDat;
}

//Ping指令，返回舵机ID，超时返回-1
int	SCS::Ping(u8 ID)
{
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, INST_PING);
	wFlushSCS();
	u8Status = 0;
	if(!checkHead()){
		u8Error = ERR_NO_REPLY;
		return -1;
	}
	u8 bBuf[4];
	u8Error = 0;
	if(readSCS(bBuf, 4)!=4){
		u8Error = ERR_NO_REPLY;
		return -1;
	}
	if(bBuf[0]!=ID && ID!=0xfe){
		u8Error = ERR_SLAVE_ID;
		return -1;
	}
	if(bBuf[1]!=2){
		u8Error = ERR_BUFF_LEN;
		return -1;
	}
	u8 calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
	if(calSum!=bBuf[3]){
		u8Error = ERR_CRC_CMP;
		return -1;			
	}
	u8Status = bBuf[2];
	return bBuf[0];
}

int SCS::checkHead()
{
	u8 bDat;
	u8 bBuf[] = {0, 0};
	u8 Cnt = 0;
	while(1){
		if(!readSCS(&bDat, 1)){
			return 0;
		}
		bBuf[1] = bBuf[0];
		bBuf[0] = bDat;
		if(bBuf[0]==0xff && bBuf[1]==0xff){
			break;
		}
		Cnt++;
		if(Cnt>10){
			return 0;
		}
	}
	return 1;
}

int	SCS::Ack(u8 ID)
{
	u8Error = 0;
	if(ID!=0xfe && Level){
		if(!checkHead()){
			u8Error = ERR_NO_REPLY;
			return 0;
		}
		u8Status = 0;
		u8 bBuf[4];
		if(readSCS(bBuf, 4)!=4){
			u8Error = ERR_NO_REPLY;
			return 0;
		}
		if(bBuf[0]!=ID){
			u8Error = ERR_SLAVE_ID;
			return 0;
		}
		if(bBuf[1]!=2){
			u8Error = ERR_BUFF_LEN;
			return 0;
		}
		u8 calSum = ~(bBuf[0]+bBuf[1]+bBuf[2]);
		if(calSum!=bBuf[3]){
			u8Error = ERR_CRC_CMP;
			return 0;
		}
		u8Status = bBuf[2];
	}
	return 1;
}

int	SCS::syncReadPacketTx(u8 ID[], u8 IDN, u8 MemAddr, u8 nLen)
{
	const int txResult = syncReadPacketTxOnly(ID, IDN, MemAddr, nLen);
	if(txResult <= 0){
		return txResult;
	}
	// 保留厂商同步接口供示例使用；P4正式链路使用TxOnly+CollectAvailable。
	syncReadRxBuffLen = readSCS(syncReadRxBuff, syncReadRxBuffMax, syncTimeOut);
	return syncReadRxBuffLen;
}

int SCS::syncReadPacketTxOnly(u8 ID[], u8 IDN, u8 MemAddr, u8 nLen)
{
	// syncReadBegin 已发现容量不足时，不发送必然无法完整接收的广播请求。
	if(!ID || IDN == 0 || syncReadRxBuffMax == 0){
		u8Error = ERR_BUFF_LEN;
		return 0;
	}
	rFlushSCS();
	syncReadRxPacketLen = nLen;
	u8 checkSum = (4+0xfe) + IDN + MemAddr + nLen + INST_SYNC_READ;
	u8 i;
	writeSCS(0xff);
	writeSCS(0xff);
	writeSCS(0xfe);
	writeSCS(IDN+4);
	writeSCS(INST_SYNC_READ);
	writeSCS(MemAddr);
	writeSCS(nLen);
	for(i=0; i<IDN; i++){
		writeSCS(ID[i]);
		checkSum += ID[i];
	}
	checkSum = ~checkSum;
	writeSCS(checkSum);
	wFlushSCS();
	return IDN + 8;
}

int SCS::syncReadCollectAvailable()
{
	if(syncReadRxBuffMax == 0 || syncReadRxBuffLen >= syncReadRxBuffMax){
		return 0;
	}
	const u16 remaining = syncReadRxBuffMax - syncReadRxBuffLen;
	const int received = readSCSAvailable(
		syncReadRxBuff + syncReadRxBuffLen, remaining);
	if(received > 0){
		syncReadRxBuffLen += (u16)received;
	}
	return received;
}

void SCS::syncReadBegin(u8 IDN, u8 rxLen, u32 TimeOut)
{
	const u16 requestedBytes = (u16)IDN * ((u16)rxLen + 6U);
	syncReadRxBuffMax =
		(requestedBytes <= kSyncReadRxBufferCapacity) ? requestedBytes : 0;
	syncReadRxBuffLen = 0;
	syncTimeOut = TimeOut;
}

void SCS::syncReadEnd()
{
	syncReadRxBuffLen = 0;
	syncReadRxBuffMax = 0;
	syncReadRxPacket = NULL;
	syncReadRxPacketIndex = 0;
}

int SCS::syncReadPacketRx(u8 ID, u8 *nDat)
{
	syncReadRxPacket = nDat;
	syncReadRxPacketIndex = 0;
	readDiagnostics = {};
	u8Status = 0;
	u8Error = ERR_NO_REPLY;
	for(u16 offset = 0; offset + 3U <= syncReadRxBuffLen; ++offset){
		if(syncReadRxBuff[offset] != 0xff || syncReadRxBuff[offset + 1] != 0xff ||
		   syncReadRxBuff[offset + 2] != ID){
			continue;
		}
		// A matching header may be the last, incomplete reply in this batch.
		if(offset + 5U > syncReadRxBuffLen) return 0;
		readDiagnostics.replyLength = syncReadRxBuff[offset + 3];
		if(readDiagnostics.replyLength != syncReadRxPacketLen + 2){
			u8Error = ERR_BUFF_LEN;
			continue;
		}
		u8Status = syncReadRxBuff[offset + 4];
		const u16 available = syncReadRxBuffLen - offset - 5U;
		readDiagnostics.payloadBytes = available < syncReadRxPacketLen
			? available : syncReadRxPacketLen;
		if(available < (u16)syncReadRxPacketLen + 1U){
			u8Error = ERR_NO_REPLY;
			return 0;
		}
		u8 calSum = ID + readDiagnostics.replyLength + u8Status;
		for(u8 i = 0; i < syncReadRxPacketLen; ++i){
			syncReadRxPacket[i] = syncReadRxBuff[offset + 5U + i];
			calSum += syncReadRxPacket[i];
		}
		calSum = ~calSum;
		if(calSum != syncReadRxBuff[offset + 5U + syncReadRxPacketLen]){
			u8Error = ERR_CRC_CMP;
			return 0;
		}
		u8Error = 0;
		readDiagnostics.statusValid = true;
		return syncReadRxPacketLen;
	}
	return 0;
}

int SCS::syncReadRxPacketToByte()
{
	if(syncReadRxPacketIndex>=syncReadRxPacketLen){
		u8Error = ERR_BUFF_LEN;
		return -1;
	}
	return syncReadRxPacket[syncReadRxPacketIndex++];
}

int SCS::syncReadRxPacketToWrod(u8 negBit)
{
	if((syncReadRxPacketIndex+1)>=syncReadRxPacketLen){
		u8Error = ERR_BUFF_LEN;
		return -1;
	}
	int Word = SCS2Host(syncReadRxPacket[syncReadRxPacketIndex], syncReadRxPacket[syncReadRxPacketIndex+1]);
	syncReadRxPacketIndex += 2;
	if(negBit){
		if(Word&(1<<negBit)){
			Word = -(Word & ~(1<<negBit));
		}
	}
	return Word;
}

int SCS::Reset(u8 ID)
{
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, INST_RESET);
	wFlushSCS();
	return Ack(ID);
}

int SCS::Recal(u8 ID)
{
	rFlushSCS();
	writeBuf(ID, 0, NULL, 0, INST_CAL);
	wFlushSCS();
	return Ack(ID);
}

int SCS::Recal(u8 ID, u16 EncodedPosition)
{
	u8Status = 0;
	u8Error = 0;
	if(ID == 0xff){
		u8Error = ERR_SLAVE_ID;
		return 0;
	}
	// Position correction has two parameters, without writeBuf's MemAddr byte.
	u8 frame[8] = {0xff, 0xff, ID, 4, INST_CAL,
		(u8)(EncodedPosition & 0xff), (u8)(EncodedPosition >> 8), 0};
	frame[7] = (u8)~(frame[2] + frame[3] + frame[4] + frame[5] + frame[6]);
	rFlushSCS();
	const int sent = writeSCS(frame, sizeof(frame));
	wFlushSCS();
	if(sent != (int)sizeof(frame)){
		u8Error = ERR_NO_REPLY;
		return 0;
	}
	if(ID == 0xfe){
		return 1; // Broadcast has no ACK; success means the full frame was sent.
	}
	// This operation requires confirmation even when the normal reply Level is 0.
	if(!checkHead()){
		u8Error = ERR_NO_REPLY;
		return 0;
	}
	u8 response[4];
	if(readSCS(response, sizeof(response)) != (int)sizeof(response)){
		u8Error = ERR_NO_REPLY;
		return 0;
	}
	if(response[0] != ID){
		u8Error = ERR_SLAVE_ID;
		return 0;
	}
	if(response[1] != 2){
		u8Error = ERR_BUFF_LEN;
		return 0;
	}
	const u8 checksum = (u8)~(response[0] + response[1] + response[2]);
	if(response[3] != checksum){
		u8Error = ERR_CRC_CMP;
		return 0;
	}
	u8Status = response[2];
	return u8Status == 0 ? 1 : 0;
}
