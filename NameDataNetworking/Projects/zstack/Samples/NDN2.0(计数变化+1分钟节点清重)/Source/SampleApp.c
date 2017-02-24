/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "MT_UART.h"
#include "MT_APP.h"
#include "MT.h"
#include "stdlib.h"

int NodeId = 3;
int layer = 2;

uint16 flag;
uint16 packagecount;
uint16 countflag;
uint16 removeflag;

static int num;//数据包总个数

//三大表的变量声明定义
/////////////////////////////////////////////////////////////////////////////////
int temp_i;	
int temp_j;
//counter记录本节点已经发送过得批次的最大值，用于过滤返回数据的个数，即聚合作用
int counter;

//Interesting相关变量定义	
InterestName	tempInterName;
Interest	AcceptInterest;
uint16 InterestLastID;
	
//Data相关变量定义
Data	AcceptData;

//CS表需要的相关定义
//CS表
CSItem	ContentStore[ContentStoreNumber];
//CS目前的条目计数
int CScounter=0;

//PIT表相关变量定义	
//PIT表	
PITItem	PendingTable[PendingNumber];
//PIT表目前的条目计数
int	PITCounter = 0;

//FIB表相关变量定义
//FIB表
FIBItem	ForwardingTable[ForwardingNumber];
int	FIBCounter = 0;//FIB表目前的条目计数
int	InsertNewName;//0为未初始化，1为插入了一个新的interestname
//////////////////////////////////////////////////////////////////////

const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_FLASH_CLUSTERID
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

endPointDesc_t SampleApp_epDesc;


uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr; //广播
afAddrType_t SampleApp_Flash_DstAddr;    //组播
afAddrType_t SampleApp_P2P_DstAddr;      //点播

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_Send_P2P_Message(void);


//NDN函数声明
/////////////////////////////////////////////////////////
void SampleApp_COOR_Send_Message(void);//协调器下发兴趣包
void SampleApp_Interest_Process(afIncomingMSGPacket_t *pkt);//兴趣包处理
void SampleApp_ContinueSend(void);
void ForwardingInterestFromFIB(int id);//FIB转发兴趣包
void SampleApp_AckMessage_Send(int lastid);
void SampleApp_Ack_Process( afIncomingMSGPacket_t *pkt );//ACK包处理
void SampleApp_DataMessage_Send(int id,int counter);//回数据包
uint16 getADC (void);//采集电压
void SampleApp_Data_Process( afIncomingMSGPacket_t *pkt );
void SendDataFromPIT( uint8 num, Data data);//PIT表发数据
void SampleApp_SendDataFromCS( uint8 num,uint8 Addr);

//NDN CS表相关声明
void InsertContentStore(Data data);
int ContentStoreFinder(Interest interest);
void DeleteItemCS(void);

//NDN PIT表相关操作函数
int PendingTableFindInterest(Interest interest);//PIT表中interest查找
void InsertNamePIT(Interest interest);//PIT表中未找到interestname则插入interestname和incommingQueue
void InsertIncomingPIT(uint8 i);//PIT表中找到了interestName则插入incommingQuere
int PendingTableFindData(Data data);//PIT表中查找Data知否为等待的data
void DeleteItemPITS(int i);//PIT表中删除特定的Item
void DeleteItemPIT(void);//PIT表中删除操作，根据已经存在的time，删除长时间的

//FIB表相关操作
//FIB表查找interest，使用模糊查询，即只是比较DestinationID和DataFlag即可，不需要需要批次一样。
int ForwardingTableFinder(Interest interest);
//FIB表未找到interest，插入新的interestnName，其他等待回复ACK后再添加。后进行广播转发
void InsertInterestFIB(Interest interest);
//FIB发送完新的interest后收到了ACK来进行outcomingQueue添加,
//若没有接受到下一跳的回复，则新的interest丢弃，根据OutcomingNumber来判断
void InsertQueueFIB(uint8 interestID,uint16 nodeid);
//FIB表进行删除，删除当前没有被重复使用且时间最长的条目
void DeleteItemFIB(void);
//FIB表中删除特定的Item
void DeleteItemFIBS(int i);

/////////////////////////////////////////////////////////////////////////////////////



void SampleApp_Init( uint8 task_id )
{ 
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  
  MT_UartInit();                  //串口初始化
  MT_UartRegisterTaskID(task_id); //注册串口任务
  
 #if defined ( BUILD_ALL_DEVICES )
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif

  // Setup for the periodic message's destination address
  // Broadcast to everyone
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;//广播
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;

  // Setup for the flash command's destination address - Group 1
  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;//组播
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;
  
  SampleApp_P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit; //点播 
  SampleApp_P2P_DstAddr.endPoint = SAMPLEAPP_ENDPOINT; 
  SampleApp_P2P_DstAddr.addr.shortAddr = 0x0000;            //发给协调器

  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SampleApp_TaskID );

  // By default, all devices start out in Group 1
  SampleApp_Group.ID = 0x0001;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7  );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SampleApp", HAL_LCD_LINE_1 );
#endif
}

//事件总和
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        // Received when a key is pressed
        case KEY_CHANGE:
          break;
          // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( SampleApp_NwkState == DEV_ZB_COORD )
          {
            // Start sending the periodic message in a regular interval.
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_COOR_COUNT_MSG_EVT,
                              SAMPLEAPP_SEND_COOR_PERIODIC_MSG_TIMEOUT );
          }
          if( SampleApp_NwkState == DEV_END_DEVICE)
          {
             osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_DuplicateRemoveCount_MSG_EVT,
                              SAMPLEAPP_SEND_COOR_PERIODIC_MSG_TIMEOUT );
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  //计数事件
  if( events & SAMPLEAPP_SEND_COOR_COUNT_MSG_EVT )
  {
     if(flag<5)
     { 
         flag++;
         osal_start_timerEx( SampleApp_TaskID,
                    SAMPLEAPP_SEND_COOR_COUNT_MSG_EVT,
                    SAMPLEAPP_SEND_COOR_PERIODIC_MSG_TIMEOUT );
     }
     else
     {
        flag=0;
        osal_start_timerEx( SampleApp_TaskID,
                    SAMPLEAPP_SEND_COOR_PERIODIC_MSG_EVT,
                    SAMPLEAPP_SEND_COOR_PERIODIC_MSG_TIMEOUT );
     }
     return(events^SAMPLEAPP_SEND_COOR_COUNT_MSG_EVT);
  }
  
  //协调器发送兴趣包事件
  if ( events & SAMPLEAPP_SEND_COOR_PERIODIC_MSG_EVT )
  {
    if( countflag <6 )
    {
      countflag++;
    }
    else if( countflag == 6 )
    {
       packagecount++;
       countflag=0;
    }
      
    SampleApp_COOR_Send_Message();

    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_COOR_COUNT_MSG_EVT,
        (SAMPLEAPP_SEND_COOR_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );

    return (events ^ SAMPLEAPP_SEND_COOR_PERIODIC_MSG_EVT);
  }
  //节点三大表去重事件
  if( events & SAMPLEAPP_DuplicateRemoveCount_MSG_EVT)
  {
     if(removeflag<60)
     {
        removeflag++;
        osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_DuplicateRemoveCount_MSG_EVT,
                              SAMPLEAPP_SEND_COOR_PERIODIC_MSG_TIMEOUT );
     }
     else
     {
       removeflag=0;
       osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_DuplicateRemove_MSG_EVT,
                              SAMPLEAPP_SEND_COOR_PERIODIC_MSG_TIMEOUT );
     }
     return( events ^ SAMPLEAPP_DuplicateRemoveCount_MSG_EVT );  
  }
  if( events & SAMPLEAPP_DuplicateRemove_MSG_EVT )
  {
     DeleteItemCS();
     DeleteItemPIT();
     DeleteItemFIB();    
     osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_DuplicateRemoveCount_MSG_EVT,
                              SAMPLEAPP_SEND_COOR_PERIODIC_MSG_TIMEOUT );
     return( events ^ SAMPLEAPP_DuplicateRemove_MSG_EVT );
  }

  // Discard unknown events
  return 0;
}

//消息处理函数
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
    case SAMPLEAPP_P2P_CLUSTERID:
      break;    
    case SAMPLEAPP_PERIODIC_CLUSTERID:
      break;
    case SAMPLEAPP_FLASH_CLUSTERID:
      break;
    case SAMPLEAPP_INTEREST_CLUSTERID:
      SampleApp_Interest_Process(pkt);
      break;
    case SAMPLEAPP_ACK_CLUSTERID:
      SampleApp_Ack_Process(pkt);
      break;
    case SAMPLEAPP_DATA_CLUSTERID:
      if( pkt->cmd.Data[10] == 0 )
      {
        HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength);
      }
      else
      {
        SampleApp_Data_Process(pkt);
      }
       break;
  }
}

//协调器下发兴趣包
void SampleApp_COOR_Send_Message(void)
{
  uint8 interestData[12];
  interestData[0] = 0x7E;
  interestData[1] = 0x45;
  interestData[2] = 1;//packageflag,interest为1，data为2，其他为3
  interestData[3] = 3;//interestName,DestinationID
  interestData[4] = 4;//interestName,DataFlag，温度=1，湿度=2，光照=3，电压=4
  interestData[5] = packagecount;//interestName,PackageCounter,包的批次号
  interestData[6] = NodeId;//lastID;
  interestData[7] = rand()%20;//Noce,随机值
  interestData[8] = 0;//保留位
  interestData[9] = 0;
  interestData[10] = layer;//层数 
  interestData[11] = 0x7E;
  
  HalUARTWrite(0, interestData, 12);
  
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_INTEREST_CLUSTERID,
                       12,
                       interestData,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  { }
}

void SampleApp_Interest_Process(afIncomingMSGPacket_t *pkt)
{
    if( pkt->cmd.Data[2]==1 && ( pkt->cmd.Data[10] == layer-1) )//兴趣包，并且是上一层下发的兴趣包
    {
       //HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength); //输出接收到的数据
       AcceptInterest.PackageFlag = 1;
       AcceptInterest.interestName.DestinationID=pkt->cmd.Data[3];
       AcceptInterest.interestName.DataFlag=pkt->cmd.Data[4];
       AcceptInterest.interestName.PackageCounter=pkt->cmd.Data[5];
       AcceptInterest.LastID=pkt->cmd.Data[6];
       AcceptInterest.Noce=pkt->cmd.Data[7];
       //判断是否回ACK包，若是协调器下发的兴趣包则不回，如果不是则回
       if( AcceptInterest.LastID!=0 )
       {
          SampleApp_AckMessage_Send(AcceptInterest.LastID);
       }
       //判断目的节点是否是自己
       if( AcceptInterest.interestName.DestinationID == NodeId )
       {
          //若是给自己，直接给上一跳节点回数据
           SampleApp_DataMessage_Send(AcceptInterest.LastID,AcceptInterest.interestName.PackageCounter);
       }
       else
       {
          //CS表中查找
          if(ContentStoreFinder(AcceptInterest)== CScounter ) //CS表查找失败
          {
             //若在CS表中查找失败，就进而在PIT表中进行查找
             if( PendingTableFindInterest(AcceptInterest) == PITCounter ) //PIT表查找失败
             {
                 //若不存在,把兴趣包存入PIT表中，然后交给FIB处理
                 InsertNamePIT( AcceptInterest );
                 //插入PIT后，在FIB中进行转发
                 if( ForwardingTableFinder(AcceptInterest) == FIBCounter )//FIB表查找失败
                 {
                       //插入FIB表
                       InsertInterestFIB(AcceptInterest); 
                       //继续向下发兴趣包
                       SampleApp_ContinueSend();
                       InsertNewName = 1;
                 }
                 else
                 {
                       //如果能在FIB表中找到，则从FIB表中进行转发
                       uint8 result = ForwardingTableFinder(AcceptInterest);
                       ForwardingInterestFromFIB(result);
                 }     
              }
             //PIT表查找成功
             else
             {
                //如果CS表中没有数据，继续广播
                if(CScounter == 0)
                {
                   SampleApp_ContinueSend();
                } 
                //在PIT表中查找到发送过来的兴趣包，则interest请求包中所对应的节点号，
                //存入PIT表中所对应的interest项的IncomingQueue队列中
                uint8 result = PendingTableFindInterest(AcceptInterest);
                InsertIncomingPIT(result);
             }
          }
          //CS表查找成功，从CS表中回传数据
          else
          {
              uint8 result = ContentStoreFinder(AcceptInterest);
              SampleApp_SendDataFromCS( result, AcceptInterest.LastID );
          }
       }
       
       
    }
}
//ack包处理
void SampleApp_Ack_Process( afIncomingMSGPacket_t *pkt )
{
   if(InsertNewName==1)	
   {
      uint8 result;
      uint8 FIB_ID = pkt->cmd.Data[3];
      result=ForwardingTableFinder(AcceptInterest);	
      InsertQueueFIB(result,FIB_ID);	
      InsertNewName=0;
   } 
   else
   {	
   //InsertNewName没有标注的话不做任何处理
   }
}

void SampleApp_Data_Process( afIncomingMSGPacket_t *pkt )
{
    AcceptData.PackageFlag = 2;
    AcceptData.interestName.DestinationID= pkt->cmd.Data[3];
    AcceptData.interestName.DataFlag= pkt->cmd.Data[4];
    AcceptData.interestName.PackageCounter= pkt->cmd.Data[5];
    AcceptData.PackageAll= pkt->cmd.Data[6];
    AcceptData.PackageSeg= pkt->cmd.Data[7];
    AcceptData.Data= BUILD_UINT16(pkt->cmd.Data[9],pkt->cmd.Data[8]);//电压值
    
    // 查询PIT表，查询表中是否有相应兴趣名存在
    if( PendingTableFindData(AcceptData) == PITCounter) //当表中不存在相应兴趣名时直接丢包
    { }
    else
    {
       uint8 result = PendingTableFindData(AcceptData);
       //从PIT表中进行转发
       SendDataFromPIT(result,AcceptData);
       //删除PIT中所对应的表项
       DeleteItemPITS(result);
       //插入数据到CS表中
       if(CScounter == 0)
       {
           InsertContentStore(AcceptData);
       }
       else
       {
           for(int i=0;i<CScounter;i++)
           {
               if( ContentStore[i].interestName.DestinationID ==AcceptData.interestName.DestinationID &&
                   ContentStore[i].interestName.DataFlag ==AcceptData.interestName.DataFlag &&
                   ContentStore[i].interestName.PackageCounter == AcceptData.interestName.PackageCounter)
                { }
                else
                {
                    DeleteItemCS();
                    InsertContentStore(AcceptData);
                }
              }
          } 
       }   
}



//继续下发兴趣包
void SampleApp_ContinueSend(void)
{
  uint8 interestData[12];
  interestData[0] = 0x7E;
  interestData[1] = 0x45;
  interestData[2] = AcceptInterest.PackageFlag;
  interestData[3] = AcceptInterest.interestName.DestinationID;
  interestData[4] = AcceptInterest.interestName.DataFlag;
  interestData[5] = AcceptInterest.interestName.PackageCounter;
  interestData[6] = NodeId;//lastID;
  interestData[7] = AcceptInterest.Noce;
  interestData[8] = 0;//保留位
  interestData[9] = 0;
  interestData[10] = layer;//层数 
  interestData[11] = 0x7E;
  
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_INTEREST_CLUSTERID,
                       12,
                       interestData,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  { }
}
//从FIB表中转发兴趣包
void ForwardingInterestFromFIB(int id)
{
   int temp = 0;
   uint8 interestData[12];
   interestData[0] = 0x7E;
   interestData[1] = 0x45;
   interestData[2] = AcceptInterest.PackageFlag;
   interestData[3] = AcceptInterest.interestName.DestinationID;
   interestData[4] = AcceptInterest.interestName.DataFlag;
   interestData[5] = AcceptInterest.interestName.PackageCounter;
   interestData[7] = AcceptInterest.Noce;
   interestData[8] = 0;//保留位
   interestData[9] = 0;
   interestData[10] = layer;//层数 
   interestData[11] = 0x7E;
   
   //被重复使用
   while(temp<ForwardingTable[id].OutcomingNumber)
   {	
       interestData[6] = ForwardingTable[id].OutcomingQueue[id];
       if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                             SAMPLEAPP_INTEREST_CLUSTERID,
                             12,
                             interestData,
                             &SampleApp_TransID,
                             AF_DISCV_ROUTE,
                             AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
         {  
         }
       temp++;
   } 
}
//从PIT表里转发数据包
void SendDataFromPIT( uint8 num, Data data)
{
   int j=0;
   uint8 ddata[12];
   ddata[0] = 0x7E;
   ddata[1] = 0x45;
   ddata[2] = 2;//PackageFlag标识包的类型，interest=1,data=2,其他=3
   //name
   ddata[3] = data.interestName.DestinationID;
   ddata[4] = data.interestName.DataFlag; //DataFlag标志需要请求的数据包的类型，温度=1，湿度=2，光照=3，电压=4
   ddata[5] = data.interestName.PackageCounter; //包批次号
   ddata[6] = data.PackageAll;//PackageAll数据包的总个数
   ddata[7] = data.PackageSeg; //PackageSeg目前属于第几个包
   ddata[8] = HI_UINT16(data.Data);
   ddata[9] = LO_UINT16(data.Data);
   ddata[10] = 0;
   ddata[11] = 0x7e;
   
   for(j=0;j<PendingTable[num].IncomingNumber;j++)
   {
      ddata[10] = PendingTable[num].IncomingQueue[j];
      if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                            SAMPLEAPP_DATA_CLUSTERID,
                            12,
                            ddata,
                            &SampleApp_TransID,
                            AF_DISCV_ROUTE,
                            AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )      
      {
      }
      else
      {
                  // Error occurred in request to send.
      }
   }   
}
//从CS表中发送数据
void SampleApp_SendDataFromCS( uint8 num,uint8 Addr)
{
   uint8 CSData[12];
   CSData[0] = 0x7E;
   CSData[1] = 0X45;
   CSData[2] = 2;
   CSData[3] = (uint8)ContentStore[num].interestName.DestinationID;
   CSData[4] = (uint8)ContentStore[num].interestName.DataFlag;
   CSData[5] = (uint8)ContentStore[num].interestName.PackageCounter;
   //CS存储其他内容
   CSData[6] = (uint8)ContentStore[num].PackageAll;
   CSData[7] = (uint8)ContentStore[num].PackageSeg;
   CSData[8] = HI_UINT16(ContentStore[num].Data);
   CSData[9] = LO_UINT16(ContentStore[num].Data);
   CSData[10] = Addr;
   CSData[11] = 0x7E;
   ++ContentStore[num].number;
   
   if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                            SAMPLEAPP_DATA_CLUSTERID,
                       12,
                       CSData,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
   {
   }
   else
   {
     // Error occurred in request to send.
   }
}

//回ACK包
void SampleApp_AckMessage_Send(int lastid)
{
  uint8 AckData[12];
  AckData[0] = 0x7E;
  AckData[1] = 0x45;
  AckData[2] = 3;//Ack为3
  AckData[3] = NodeId;
  AckData[4] = lastid;
  AckData[5] = 0;
  AckData[6] = 0;
  AckData[7] = 0;
  AckData[8] = 0;
  AckData[9] = 0;
  AckData[10] = 0;
  AckData[11] = 0x7E;
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_ACK_CLUSTERID,
                       12,
                       AckData,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
   {
   }
   else
   { }     
}
//直接回数据包
void SampleApp_DataMessage_Send(int id,int counter)
{
   num++;
   uint8 data[12];
   data[0] = 0x7E;
   data[1] = 0x45;
   data[2] = 2;//PackageFlag标识包的类型，interest=1,data=2,其他=3
   //name
   data[3] = NodeId;
   data[4] = 4; //DataFlag标志需要请求的数据包的类型，温度=1，湿度=2，光照=3，电压=4
   data[5] = counter; //包批次号
   data[6] = num;//PackageAll数据包的总个数
   data[7] = num; //PackageSeg目前属于第几个包
   uint16 voltage = getADC();
   data[8] = HI_UINT16(voltage);
   data[9] = LO_UINT16(voltage);
   data[10] = id;//数据包的目的id
   data[11] = 0x7e;
   
   if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                            SAMPLEAPP_DATA_CLUSTERID,
                       12,
                       data,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
   {
   }
   else
   {
       // Error occurred in request to send.
   }
}



void SampleApp_Send_P2P_Message( void )
{
  uint8 data[11]="0123456789";
  
  if ( AF_DataRequest( &SampleApp_P2P_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_P2P_CLUSTERID,
                       10,
                       data,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}
/*********************************************************************
*********************************************************************/


/*********************************************************************
*********************************************************************/
//NDN相关表的操作
//插入CS表
void InsertContentStore(Data data)
{
  uint8 temp;	
 //若CS表未满
  if(CScounter<10)
  {
    //CS存储Interest相关内容
    ContentStore[CScounter].interestName.DestinationID=data.interestName.DestinationID;
    ContentStore[CScounter].interestName.DataFlag=data.interestName.DataFlag;	
    ContentStore[CScounter].interestName.PackageCounter=data.interestName.PackageCounter;
    //CS存储其他内容
    ContentStore[CScounter].PackageAll=data.PackageAll;
    ContentStore[CScounter].PackageSeg=data.PackageSeg;
    ContentStore[CScounter].Data=data.Data;
    ContentStore[CScounter].number=0;	
    CScounter = CScounter+1;
  }
  //若CS表已经装满，需要使用替换策略进行替换，我们使用LRU(最近最少使用)
  else
  {	
    temp=0;
    for(int temp_i=1;temp_i<ContentStoreNumber;++temp_i)
    {   
         if(ContentStore[temp].number > ContentStore[temp_i].number)
             temp=temp_i;
    }	
    //替换temp所在位置的内容	
    //CS存储Interest相关内容
    ContentStore[temp].interestName.DestinationID=data.interestName.DestinationID;	
    ContentStore[temp].interestName.DataFlag=data.interestName.DataFlag;	
    ContentStore[temp].interestName.PackageCounter=data.interestName.PackageCounter;	
    //CS存储其他内容
    ContentStore[temp].PackageAll=data.PackageAll;	
    ContentStore[temp].PackageSeg=data.PackageSeg;	
    ContentStore[temp].Data=data.Data;	
    ContentStore[temp].number=0;
  }
}

//查找CS表
int ContentStoreFinder(Interest interest)
{
  //Interst包中InterestName临时进行存储
  tempInterName.DestinationID=interest.interestName.DestinationID;
  tempInterName.DataFlag=interest.interestName.DataFlag;	
  tempInterName.PackageCounter=interest.interestName.PackageCounter;
  for(temp_i=0;temp_i<CScounter;)
  {
    temp_j=0;		
    if(tempInterName.DestinationID==ContentStore[temp_i].interestName.DestinationID)	
    {		
      ++temp_j;	
    }	
    if(tempInterName.DataFlag==ContentStore[temp_i].interestName.DataFlag)	
    {		
      ++temp_j;	
    }	
    if(tempInterName.PackageCounter==ContentStore[temp_i].interestName.PackageCounter)	
    {		
      ++temp_j;	
    }		
    if(temp_j==3)	
    {		
      return temp_i;	
    }
    temp_i++;
  }
  //如果temp_i不为CScounter则查找成功，否则查找失败
  return CScounter;
}
//从CS表中根据time删除使用次数最少的数据
void DeleteItemCS(void)
{
  int temp;
  temp=0;
  for(temp_i=0;temp_i<CScounter;++temp_i)
  {	
    if(ContentStore[temp].number>ContentStore[temp_i].number)	
    {		
      temp=temp_i;	
    }
  }
  if(ContentStore[temp].number<2 && temp<CScounter-1)
  {	
    for(temp_i=temp;temp_i<CScounter;++temp_i)	
    {		
      ContentStore[temp_i].interestName.DestinationID=ContentStore[temp_i+1].interestName.DestinationID;		
      ContentStore[temp_i].interestName.DataFlag=ContentStore[temp_i+1].interestName.DataFlag;		
      ContentStore[temp_i].interestName.PackageCounter=ContentStore[temp_i+1].interestName.PackageCounter;		
      ContentStore[temp_i].PackageAll=ContentStore[temp_i+1].PackageAll;		
      ContentStore[temp_i].PackageSeg=ContentStore[temp_i+1].PackageSeg;		
      ContentStore[temp_i].Data=ContentStore[temp_i+1].Data;		
      ContentStore[temp_i].number=ContentStore[temp_i+1].number;	
    }	
    CScounter--;
  }
  else if(ContentStore[temp].number<2 && temp==CScounter-1)
  {	
    CScounter--;
  }
}

//PIT表相关处理操作
//PIT表中interest查找
int  PendingTableFindInterest(Interest interest)
{
  //Interst包中InterestName临时进行存储
  tempInterName.DestinationID=interest.interestName.DestinationID;	
  tempInterName.DataFlag=interest.interestName.DataFlag;	
  tempInterName.PackageCounter=interest.interestName.PackageCounter;
  for(temp_i=0;temp_i<PITCounter;temp_i++)
  {	
    temp_j=0;	
    if(tempInterName.DestinationID==PendingTable[temp_i].interestName.DestinationID)	
    {		
      ++temp_j;	
    }		
    if(tempInterName.DataFlag==PendingTable[temp_i].interestName.DataFlag)	
    {		
      ++temp_j;	
    }		
    if(tempInterName.PackageCounter==PendingTable[temp_i].interestName.PackageCounter)	
    {		
      ++temp_j;	
    }		
    if(temp_j==3)	
    {		
      return temp_i;	
    }
  }
  //如果temp_i不为PITCounter则查找成功，否则查找失败
  return PITCounter;
}

//PIT表中未找到interestname则插入interestname和incommingQueue
void InsertNamePIT(Interest interest)
{
  int	temp;
  //若PIT表未满
  if(PITCounter<PendingNumber)
  {	
    //PIT存储Interest相关内容
    PendingTable[PITCounter].interestName.DestinationID=interest.interestName.DestinationID;	
    PendingTable[PITCounter].interestName.DataFlag=interest.interestName.DataFlag;	
    PendingTable[PITCounter].interestName.PackageCounter=interest.interestName.PackageCounter;	
    //PIT存储其他内容
    PendingTable[PITCounter].IncomingQueue[PendingTable[PITCounter].IncomingNumber]=AcceptInterest.LastID;	
    PendingTable[PITCounter].IncomingNumber++;	
    PendingTable[PITCounter].time=0;//time需要定时的来更新，后续处理	
    PITCounter++;
  }
    //若PIT表已经装满，需要使用替换策略进行替换，我们淘汰掉最长等待时间的条目
  else
  {
     temp=0;
     for(temp_i=1;temp_i<PITCounter;++temp_i)
     {
	if(PendingTable[temp].time<PendingTable[temp_i].time)			
         temp=temp_i;
     }
     //替换temp所在位置的内容r
     //PIT存储Interest相关内容
     PendingTable[temp].interestName.DestinationID=interest.interestName.DestinationID;
     PendingTable[temp].interestName.DataFlag=interest.interestName.DataFlag;
     PendingTable[temp].interestName.PackageCounter=interest.interestName.PackageCounter;
     //PIT存储其他内容
     PendingTable[temp].IncomingQueue[PendingTable[temp].IncomingNumber]=AcceptInterest.LastID;	
     PendingTable[temp].IncomingNumber++;
     PendingTable[temp].time=0;
    }
}

//PIT表中找到了interestName则插入incommingQuere
void InsertIncomingPIT(uint8 i)
{
  //目前只存储前5个到达的interest包
  if(PendingTable[i].IncomingNumber<5)
  {	
    PendingTable[i].IncomingQueue[PendingTable[i].IncomingNumber]=AcceptInterest.LastID;	
    PendingTable[i].IncomingNumber++;  // pitcounter 替换为 i
  }
}

//PIT表中查找Data知否为等待的data
int PendingTableFindData(Data data)
{
  //Interst包中InterestName临时进行存储
  tempInterName.DestinationID=data.interestName.DestinationID;
  tempInterName.DataFlag=data.interestName.DataFlag;
  tempInterName.PackageCounter=data.interestName.PackageCounter;
  for(temp_i=0;temp_i<PITCounter;temp_i++)
  {	
    temp_j=0;		
    if(tempInterName.DestinationID==PendingTable[temp_i].interestName.DestinationID)	
    {		
      ++temp_j;	
    }	
    if(tempInterName.DataFlag==PendingTable[temp_i].interestName.DataFlag)	
    {		
      ++temp_j;	
    }		
    if(tempInterName.PackageCounter==PendingTable[temp_i].interestName.PackageCounter)	
    {		
      ++temp_j;	
    }		
    if(temp_j==3)	
    {		
      return temp_i;	
    }
  }
  //如果temp_i不为PITCounter则查找成功，否则查找失败
  return PITCounter;
}

//PIT表中删除操作，根据已经存在的time，删除长时间的
void DeleteItemPIT( void )
{
  int temp;
  temp=0;
  for(temp_i=0;temp_i<PITCounter;++temp_i)
  {	
    if(PendingTable[temp].time<PendingTable[temp_i].time)	
    {		
      temp=temp_i;	
    }
  }	
  if(temp<PITCounter-1)
  {
    for(temp_i=temp;temp_i<PITCounter;++temp_i)	
    {		
      PendingTable[temp_i].interestName.DestinationID=PendingTable[temp_i+1].interestName.DestinationID;		
      PendingTable[temp_i].interestName.DataFlag=PendingTable[temp_i+1].interestName.DataFlag;		
      PendingTable[temp_i].interestName.PackageCounter=PendingTable[temp_i+1].interestName.PackageCounter;
      for(temp_j=0;temp_j<5;++temp_j)		
      {   
           PendingTable[temp_i].IncomingQueue[temp_j]=PendingTable[temp_i+1].IncomingQueue[temp_j];	
      }	
      PendingTable[temp_i].IncomingNumber=PendingTable[temp_i+1].IncomingNumber;	
      PendingTable[temp_i].time=PendingTable[temp_i+1].time;	
    }		
    PITCounter--;
  }
  else
  {	
    PITCounter--;
  }
}

//PIT表中删除特定的Item
void DeleteItemPITS(int i)
{
  if(i<PITCounter-1)
  {	
    for(temp_i=i;temp_i<PITCounter;++temp_i)	
    {		
      PendingTable[temp_i].interestName.DestinationID=PendingTable[temp_i+1].interestName.DestinationID;		
      PendingTable[temp_i].interestName.DataFlag=PendingTable[temp_i+1].interestName.DataFlag;		
      PendingTable[temp_i].interestName.PackageCounter=PendingTable[temp_i+1].interestName.PackageCounter;
      for(temp_j=0;temp_j<5;++temp_j)		
      {			
        PendingTable[temp_i].IncomingQueue[temp_j]=PendingTable[temp_i+1].IncomingQueue[temp_j];		
      }		
      PendingTable[temp_i].IncomingNumber=PendingTable[temp_i+1].IncomingNumber;		
      PendingTable[temp_i].time=PendingTable[temp_i+1].time;
    }	
      PITCounter--;
  }
  else
  {	
    PITCounter--;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
//FIB表相关操作
//FIB表查找interest，使用模糊查询，即只是比较DestinationID和DataFlag即可，不需要需要批次一样。
int ForwardingTableFinder(Interest interest)
{
  //Interst包中InterestName临时进行存储
  tempInterName.DestinationID=interest.interestName.DestinationID;
  tempInterName.DataFlag=interest.interestName.DataFlag;
  tempInterName.PackageCounter=interest.interestName.PackageCounter;
  for(temp_i=0;temp_i<FIBCounter;temp_i++)
  {	
    temp_j=0;		
    if(tempInterName.DestinationID==ForwardingTable[temp_i].interestName.DestinationID)	
    {		
      ++temp_j;	
    }	
    if(tempInterName.DataFlag==ForwardingTable[temp_i].interestName.DataFlag)	
    {		
      ++temp_j;	
    }	
    if(temp_j==2)	
    {		
      return temp_i;	
    }
  }
  //如果temp_i不为FIBCounter则查找成功，否则查找失败
  return FIBCounter;
}


//FIB表未找到interest，插入新的interestnName，其他等待回复ACK后再添加。后进行广播转发
void InsertInterestFIB(Interest interest)
{
  int temp;
  //若FIB表未满
  if(FIBCounter<ForwardingNumber)
  {	
    //FIB存储Interest相关内容	
    ForwardingTable[FIBCounter].interestName.DestinationID=interest.interestName.DestinationID;	
    ForwardingTable[FIBCounter].interestName.DataFlag=interest.interestName.DataFlag;	
    ForwardingTable[FIBCounter].interestName.PackageCounter=interest.interestName.PackageCounter;	
    ForwardingTable[FIBCounter].time=0;
    //time需要定时的来更新，后续处理		
    FIBCounter++;
  }
  //若FIB表已经装满，需要使用替换策略进行替换，我们淘汰掉最长等待时间的条目
  else
  {	
    temp=0;		
    for(temp_i=1;temp_i<FIBCounter;++temp_i)	
    {		
      if(ForwardingTable[temp].time<ForwardingTable[temp_i].time && ForwardingTable[temp].used==0 && ForwardingTable[temp_i].used==0)	
        temp=temp_i;	
    }		
    //替换temp所在位置的内容		
    //PIT存储Interest相关内容
    ForwardingTable[temp].interestName.DestinationID=interest.interestName.DestinationID;	
    ForwardingTable[temp].interestName.DataFlag=interest.interestName.DataFlag;	
    ForwardingTable[temp].interestName.PackageCounter=interest.interestName.PackageCounter;	
    ForwardingTable[temp].time=0;
  }
}


//FIB发送完新的interest后收到了ACK来进行outcomingQueue添加,
//若没有接受到下一跳的回复，则新的interest丢弃，根据OutcomingNumber来判断
void InsertQueueFIB(uint8 interestID,uint16 nodeid)
{
  //FIB进行对应的interestID的输出节点号的添加
  //判断是否有重复出口，若有则不进行添加
  for(temp_i=0;temp_i<ForwardingTable[interestID].OutcomingNumber;temp_i++)
  {	
    if(nodeid==ForwardingTable[interestID].OutcomingQueue[temp_i])					
      return ;
  }
  if(temp_i==ForwardingTable[interestID].OutcomingNumber)
  {	
    ForwardingTable[interestID].OutcomingQueue[ForwardingTable[interestID].OutcomingNumber]=nodeid; 
    ForwardingTable[interestID].OutcomingNumber++;
  }
}


//FIB表进行删除，删除当前没有被重复使用且时间最长的条目
void DeleteItemFIB(void)
{
  int temp;
  temp=0;
  for(temp_i=0;temp_i<FIBCounter;++temp_i)
  {	
    if(ForwardingTable[temp].time<ForwardingTable[temp_i].time && ForwardingTable[temp].used==0 && ForwardingTable[temp_i].used==0)	
    {		
      temp=temp_i;	
    }
  }
  if(temp<FIBCounter-1)
  {	
    for(temp_i=temp;temp_i<FIBCounter;++temp_i)	
    {	
      ForwardingTable[temp_i].interestName.DestinationID=ForwardingTable[temp_i+1].interestName.DestinationID;		
      ForwardingTable[temp_i].interestName.DataFlag=ForwardingTable[temp_i+1].interestName.DataFlag;		
      ForwardingTable[temp_i].interestName.PackageCounter=ForwardingTable[temp_i+1].interestName.PackageCounter;
      for(temp_j=0;temp_j<5;++temp_j)	
      {		
        ForwardingTable[temp_i].OutcomingQueue[temp_j]=ForwardingTable[temp_i+1].OutcomingQueue[temp_j];	
      }		
      ForwardingTable[temp_i].OutcomingNumber=ForwardingTable[temp_i+1].OutcomingNumber;
      ForwardingTable[temp_i].time=ForwardingTable[temp_i+1].time;	
      ForwardingTable[temp_i].used=ForwardingTable[temp_i+1].used;	
    }	
    FIBCounter--;
  }
  else
  {	
    FIBCounter--;
  }
}

//FIB表中删除特定的Item
void DeleteItemFIBS(int i)
{
  if(ForwardingTable[i].used>0)
  {
    ForwardingTable[i].used--;
  }
  else
  {	
    if(i<FIBCounter-1)
    {	
      for(temp_i=i;temp_i<FIBCounter;++temp_i)	
      {			
        ForwardingTable[temp_i].interestName.DestinationID=ForwardingTable[temp_i+1].interestName.DestinationID;			
        ForwardingTable[temp_i].interestName.DataFlag=ForwardingTable[temp_i+1].interestName.DataFlag;			
        ForwardingTable[temp_i].interestName.PackageCounter=ForwardingTable[temp_i+1].interestName.PackageCounter;	
	for(temp_j=0;temp_j<5;++temp_j)			
        {			
          ForwardingTable[temp_i].OutcomingQueue[temp_j]=ForwardingTable[temp_i+1].OutcomingQueue[temp_j];		
        }				
        ForwardingTable[temp_i].OutcomingNumber=ForwardingTable[temp_i+1].OutcomingNumber;			
        ForwardingTable[temp_i].time=ForwardingTable[temp_i+1].time;			
        ForwardingTable[temp_i].used=ForwardingTable[temp_i+1].used;
      }		
      FIBCounter--;	
    }	
    else	
    {		
      FIBCounter--;	
    }
  }
}

uint16 getADC (void)
{
  uint16 value;
  uint8 tmpADCCON3 = ADCCON3;  // Save ADCCON3 to restore later

  /* Clear ADC interrupt flag */
  ADCIF = 0;

  /* Setup the new value for conversion */
  ADCCON3 = (0x00 | 0x00 | 0x0f);

  /* Wait for the conversion to finish */
  while ( !ADCIF );

  /* Get the result */
  value = ADCL;
  value |= ((uint16) ADCH) << 8;

  // Restore ADCCON3
  ADCCON3 = tmpADCCON3;
  
  /* Check the limit and return */
  return value;
}