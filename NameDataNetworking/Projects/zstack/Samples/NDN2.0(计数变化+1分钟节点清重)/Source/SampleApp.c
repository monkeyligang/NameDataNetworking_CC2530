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

static int num;//���ݰ��ܸ���

//�����ı�����������
/////////////////////////////////////////////////////////////////////////////////
int temp_i;	
int temp_j;
//counter��¼���ڵ��Ѿ����͹������ε����ֵ�����ڹ��˷������ݵĸ��������ۺ�����
int counter;

//Interesting��ر�������	
InterestName	tempInterName;
Interest	AcceptInterest;
uint16 InterestLastID;
	
//Data��ر�������
Data	AcceptData;

//CS����Ҫ����ض���
//CS��
CSItem	ContentStore[ContentStoreNumber];
//CSĿǰ����Ŀ����
int CScounter=0;

//PIT����ر�������	
//PIT��	
PITItem	PendingTable[PendingNumber];
//PIT��Ŀǰ����Ŀ����
int	PITCounter = 0;

//FIB����ر�������
//FIB��
FIBItem	ForwardingTable[ForwardingNumber];
int	FIBCounter = 0;//FIB��Ŀǰ����Ŀ����
int	InsertNewName;//0Ϊδ��ʼ����1Ϊ������һ���µ�interestname
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

afAddrType_t SampleApp_Periodic_DstAddr; //�㲥
afAddrType_t SampleApp_Flash_DstAddr;    //�鲥
afAddrType_t SampleApp_P2P_DstAddr;      //�㲥

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_Send_P2P_Message(void);


//NDN��������
/////////////////////////////////////////////////////////
void SampleApp_COOR_Send_Message(void);//Э�����·���Ȥ��
void SampleApp_Interest_Process(afIncomingMSGPacket_t *pkt);//��Ȥ������
void SampleApp_ContinueSend(void);
void ForwardingInterestFromFIB(int id);//FIBת����Ȥ��
void SampleApp_AckMessage_Send(int lastid);
void SampleApp_Ack_Process( afIncomingMSGPacket_t *pkt );//ACK������
void SampleApp_DataMessage_Send(int id,int counter);//�����ݰ�
uint16 getADC (void);//�ɼ���ѹ
void SampleApp_Data_Process( afIncomingMSGPacket_t *pkt );
void SendDataFromPIT( uint8 num, Data data);//PIT������
void SampleApp_SendDataFromCS( uint8 num,uint8 Addr);

//NDN CS���������
void InsertContentStore(Data data);
int ContentStoreFinder(Interest interest);
void DeleteItemCS(void);

//NDN PIT����ز�������
int PendingTableFindInterest(Interest interest);//PIT����interest����
void InsertNamePIT(Interest interest);//PIT����δ�ҵ�interestname�����interestname��incommingQueue
void InsertIncomingPIT(uint8 i);//PIT�����ҵ���interestName�����incommingQuere
int PendingTableFindData(Data data);//PIT���в���Data֪��Ϊ�ȴ���data
void DeleteItemPITS(int i);//PIT����ɾ���ض���Item
void DeleteItemPIT(void);//PIT����ɾ�������������Ѿ����ڵ�time��ɾ����ʱ���

//FIB����ز���
//FIB�����interest��ʹ��ģ����ѯ����ֻ�ǱȽ�DestinationID��DataFlag���ɣ�����Ҫ��Ҫ����һ����
int ForwardingTableFinder(Interest interest);
//FIB��δ�ҵ�interest�������µ�interestnName�������ȴ��ظ�ACK������ӡ�����й㲥ת��
void InsertInterestFIB(Interest interest);
//FIB�������µ�interest���յ���ACK������outcomingQueue���,
//��û�н��ܵ���һ���Ļظ������µ�interest����������OutcomingNumber���ж�
void InsertQueueFIB(uint8 interestID,uint16 nodeid);
//FIB�����ɾ����ɾ����ǰû�б��ظ�ʹ����ʱ�������Ŀ
void DeleteItemFIB(void);
//FIB����ɾ���ض���Item
void DeleteItemFIBS(int i);

/////////////////////////////////////////////////////////////////////////////////////



void SampleApp_Init( uint8 task_id )
{ 
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  
  MT_UartInit();                  //���ڳ�ʼ��
  MT_UartRegisterTaskID(task_id); //ע�ᴮ������
  
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
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;//�㲥
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;

  // Setup for the flash command's destination address - Group 1
  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;//�鲥
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;
  
  SampleApp_P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit; //�㲥 
  SampleApp_P2P_DstAddr.endPoint = SAMPLEAPP_ENDPOINT; 
  SampleApp_P2P_DstAddr.addr.shortAddr = 0x0000;            //����Э����

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

//�¼��ܺ�
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
  //�����¼�
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
  
  //Э����������Ȥ���¼�
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
  //�ڵ������ȥ���¼�
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

//��Ϣ������
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

//Э�����·���Ȥ��
void SampleApp_COOR_Send_Message(void)
{
  uint8 interestData[12];
  interestData[0] = 0x7E;
  interestData[1] = 0x45;
  interestData[2] = 1;//packageflag,interestΪ1��dataΪ2������Ϊ3
  interestData[3] = 3;//interestName,DestinationID
  interestData[4] = 4;//interestName,DataFlag���¶�=1��ʪ��=2������=3����ѹ=4
  interestData[5] = packagecount;//interestName,PackageCounter,�������κ�
  interestData[6] = NodeId;//lastID;
  interestData[7] = rand()%20;//Noce,���ֵ
  interestData[8] = 0;//����λ
  interestData[9] = 0;
  interestData[10] = layer;//���� 
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
    if( pkt->cmd.Data[2]==1 && ( pkt->cmd.Data[10] == layer-1) )//��Ȥ������������һ���·�����Ȥ��
    {
       //HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength); //������յ�������
       AcceptInterest.PackageFlag = 1;
       AcceptInterest.interestName.DestinationID=pkt->cmd.Data[3];
       AcceptInterest.interestName.DataFlag=pkt->cmd.Data[4];
       AcceptInterest.interestName.PackageCounter=pkt->cmd.Data[5];
       AcceptInterest.LastID=pkt->cmd.Data[6];
       AcceptInterest.Noce=pkt->cmd.Data[7];
       //�ж��Ƿ��ACK��������Э�����·�����Ȥ���򲻻أ�����������
       if( AcceptInterest.LastID!=0 )
       {
          SampleApp_AckMessage_Send(AcceptInterest.LastID);
       }
       //�ж�Ŀ�Ľڵ��Ƿ����Լ�
       if( AcceptInterest.interestName.DestinationID == NodeId )
       {
          //���Ǹ��Լ���ֱ�Ӹ���һ���ڵ������
           SampleApp_DataMessage_Send(AcceptInterest.LastID,AcceptInterest.interestName.PackageCounter);
       }
       else
       {
          //CS���в���
          if(ContentStoreFinder(AcceptInterest)== CScounter ) //CS�����ʧ��
          {
             //����CS���в���ʧ�ܣ��ͽ�����PIT���н��в���
             if( PendingTableFindInterest(AcceptInterest) == PITCounter ) //PIT�����ʧ��
             {
                 //��������,����Ȥ������PIT���У�Ȼ�󽻸�FIB����
                 InsertNamePIT( AcceptInterest );
                 //����PIT����FIB�н���ת��
                 if( ForwardingTableFinder(AcceptInterest) == FIBCounter )//FIB�����ʧ��
                 {
                       //����FIB��
                       InsertInterestFIB(AcceptInterest); 
                       //�������·���Ȥ��
                       SampleApp_ContinueSend();
                       InsertNewName = 1;
                 }
                 else
                 {
                       //�������FIB�����ҵ������FIB���н���ת��
                       uint8 result = ForwardingTableFinder(AcceptInterest);
                       ForwardingInterestFromFIB(result);
                 }     
              }
             //PIT����ҳɹ�
             else
             {
                //���CS����û�����ݣ������㲥
                if(CScounter == 0)
                {
                   SampleApp_ContinueSend();
                } 
                //��PIT���в��ҵ����͹�������Ȥ������interest�����������Ӧ�Ľڵ�ţ�
                //����PIT��������Ӧ��interest���IncomingQueue������
                uint8 result = PendingTableFindInterest(AcceptInterest);
                InsertIncomingPIT(result);
             }
          }
          //CS����ҳɹ�����CS���лش�����
          else
          {
              uint8 result = ContentStoreFinder(AcceptInterest);
              SampleApp_SendDataFromCS( result, AcceptInterest.LastID );
          }
       }
       
       
    }
}
//ack������
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
   //InsertNewNameû�б�ע�Ļ������κδ���
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
    AcceptData.Data= BUILD_UINT16(pkt->cmd.Data[9],pkt->cmd.Data[8]);//��ѹֵ
    
    // ��ѯPIT����ѯ�����Ƿ�����Ӧ��Ȥ������
    if( PendingTableFindData(AcceptData) == PITCounter) //�����в�������Ӧ��Ȥ��ʱֱ�Ӷ���
    { }
    else
    {
       uint8 result = PendingTableFindData(AcceptData);
       //��PIT���н���ת��
       SendDataFromPIT(result,AcceptData);
       //ɾ��PIT������Ӧ�ı���
       DeleteItemPITS(result);
       //�������ݵ�CS����
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



//�����·���Ȥ��
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
  interestData[8] = 0;//����λ
  interestData[9] = 0;
  interestData[10] = layer;//���� 
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
//��FIB����ת����Ȥ��
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
   interestData[8] = 0;//����λ
   interestData[9] = 0;
   interestData[10] = layer;//���� 
   interestData[11] = 0x7E;
   
   //���ظ�ʹ��
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
//��PIT����ת�����ݰ�
void SendDataFromPIT( uint8 num, Data data)
{
   int j=0;
   uint8 ddata[12];
   ddata[0] = 0x7E;
   ddata[1] = 0x45;
   ddata[2] = 2;//PackageFlag��ʶ�������ͣ�interest=1,data=2,����=3
   //name
   ddata[3] = data.interestName.DestinationID;
   ddata[4] = data.interestName.DataFlag; //DataFlag��־��Ҫ��������ݰ������ͣ��¶�=1��ʪ��=2������=3����ѹ=4
   ddata[5] = data.interestName.PackageCounter; //�����κ�
   ddata[6] = data.PackageAll;//PackageAll���ݰ����ܸ���
   ddata[7] = data.PackageSeg; //PackageSegĿǰ���ڵڼ�����
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
//��CS���з�������
void SampleApp_SendDataFromCS( uint8 num,uint8 Addr)
{
   uint8 CSData[12];
   CSData[0] = 0x7E;
   CSData[1] = 0X45;
   CSData[2] = 2;
   CSData[3] = (uint8)ContentStore[num].interestName.DestinationID;
   CSData[4] = (uint8)ContentStore[num].interestName.DataFlag;
   CSData[5] = (uint8)ContentStore[num].interestName.PackageCounter;
   //CS�洢��������
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

//��ACK��
void SampleApp_AckMessage_Send(int lastid)
{
  uint8 AckData[12];
  AckData[0] = 0x7E;
  AckData[1] = 0x45;
  AckData[2] = 3;//AckΪ3
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
//ֱ�ӻ����ݰ�
void SampleApp_DataMessage_Send(int id,int counter)
{
   num++;
   uint8 data[12];
   data[0] = 0x7E;
   data[1] = 0x45;
   data[2] = 2;//PackageFlag��ʶ�������ͣ�interest=1,data=2,����=3
   //name
   data[3] = NodeId;
   data[4] = 4; //DataFlag��־��Ҫ��������ݰ������ͣ��¶�=1��ʪ��=2������=3����ѹ=4
   data[5] = counter; //�����κ�
   data[6] = num;//PackageAll���ݰ����ܸ���
   data[7] = num; //PackageSegĿǰ���ڵڼ�����
   uint16 voltage = getADC();
   data[8] = HI_UINT16(voltage);
   data[9] = LO_UINT16(voltage);
   data[10] = id;//���ݰ���Ŀ��id
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
//NDN��ر�Ĳ���
//����CS��
void InsertContentStore(Data data)
{
  uint8 temp;	
 //��CS��δ��
  if(CScounter<10)
  {
    //CS�洢Interest�������
    ContentStore[CScounter].interestName.DestinationID=data.interestName.DestinationID;
    ContentStore[CScounter].interestName.DataFlag=data.interestName.DataFlag;	
    ContentStore[CScounter].interestName.PackageCounter=data.interestName.PackageCounter;
    //CS�洢��������
    ContentStore[CScounter].PackageAll=data.PackageAll;
    ContentStore[CScounter].PackageSeg=data.PackageSeg;
    ContentStore[CScounter].Data=data.Data;
    ContentStore[CScounter].number=0;	
    CScounter = CScounter+1;
  }
  //��CS���Ѿ�װ������Ҫʹ���滻���Խ����滻������ʹ��LRU(�������ʹ��)
  else
  {	
    temp=0;
    for(int temp_i=1;temp_i<ContentStoreNumber;++temp_i)
    {   
         if(ContentStore[temp].number > ContentStore[temp_i].number)
             temp=temp_i;
    }	
    //�滻temp����λ�õ�����	
    //CS�洢Interest�������
    ContentStore[temp].interestName.DestinationID=data.interestName.DestinationID;	
    ContentStore[temp].interestName.DataFlag=data.interestName.DataFlag;	
    ContentStore[temp].interestName.PackageCounter=data.interestName.PackageCounter;	
    //CS�洢��������
    ContentStore[temp].PackageAll=data.PackageAll;	
    ContentStore[temp].PackageSeg=data.PackageSeg;	
    ContentStore[temp].Data=data.Data;	
    ContentStore[temp].number=0;
  }
}

//����CS��
int ContentStoreFinder(Interest interest)
{
  //Interst����InterestName��ʱ���д洢
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
  //���temp_i��ΪCScounter����ҳɹ����������ʧ��
  return CScounter;
}
//��CS���и���timeɾ��ʹ�ô������ٵ�����
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

//PIT����ش������
//PIT����interest����
int  PendingTableFindInterest(Interest interest)
{
  //Interst����InterestName��ʱ���д洢
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
  //���temp_i��ΪPITCounter����ҳɹ����������ʧ��
  return PITCounter;
}

//PIT����δ�ҵ�interestname�����interestname��incommingQueue
void InsertNamePIT(Interest interest)
{
  int	temp;
  //��PIT��δ��
  if(PITCounter<PendingNumber)
  {	
    //PIT�洢Interest�������
    PendingTable[PITCounter].interestName.DestinationID=interest.interestName.DestinationID;	
    PendingTable[PITCounter].interestName.DataFlag=interest.interestName.DataFlag;	
    PendingTable[PITCounter].interestName.PackageCounter=interest.interestName.PackageCounter;	
    //PIT�洢��������
    PendingTable[PITCounter].IncomingQueue[PendingTable[PITCounter].IncomingNumber]=AcceptInterest.LastID;	
    PendingTable[PITCounter].IncomingNumber++;	
    PendingTable[PITCounter].time=0;//time��Ҫ��ʱ�������£���������	
    PITCounter++;
  }
    //��PIT���Ѿ�װ������Ҫʹ���滻���Խ����滻��������̭����ȴ�ʱ�����Ŀ
  else
  {
     temp=0;
     for(temp_i=1;temp_i<PITCounter;++temp_i)
     {
	if(PendingTable[temp].time<PendingTable[temp_i].time)			
         temp=temp_i;
     }
     //�滻temp����λ�õ�����r
     //PIT�洢Interest�������
     PendingTable[temp].interestName.DestinationID=interest.interestName.DestinationID;
     PendingTable[temp].interestName.DataFlag=interest.interestName.DataFlag;
     PendingTable[temp].interestName.PackageCounter=interest.interestName.PackageCounter;
     //PIT�洢��������
     PendingTable[temp].IncomingQueue[PendingTable[temp].IncomingNumber]=AcceptInterest.LastID;	
     PendingTable[temp].IncomingNumber++;
     PendingTable[temp].time=0;
    }
}

//PIT�����ҵ���interestName�����incommingQuere
void InsertIncomingPIT(uint8 i)
{
  //Ŀǰֻ�洢ǰ5�������interest��
  if(PendingTable[i].IncomingNumber<5)
  {	
    PendingTable[i].IncomingQueue[PendingTable[i].IncomingNumber]=AcceptInterest.LastID;	
    PendingTable[i].IncomingNumber++;  // pitcounter �滻Ϊ i
  }
}

//PIT���в���Data֪��Ϊ�ȴ���data
int PendingTableFindData(Data data)
{
  //Interst����InterestName��ʱ���д洢
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
  //���temp_i��ΪPITCounter����ҳɹ����������ʧ��
  return PITCounter;
}

//PIT����ɾ�������������Ѿ����ڵ�time��ɾ����ʱ���
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

//PIT����ɾ���ض���Item
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
//FIB����ز���
//FIB�����interest��ʹ��ģ����ѯ����ֻ�ǱȽ�DestinationID��DataFlag���ɣ�����Ҫ��Ҫ����һ����
int ForwardingTableFinder(Interest interest)
{
  //Interst����InterestName��ʱ���д洢
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
  //���temp_i��ΪFIBCounter����ҳɹ����������ʧ��
  return FIBCounter;
}


//FIB��δ�ҵ�interest�������µ�interestnName�������ȴ��ظ�ACK������ӡ�����й㲥ת��
void InsertInterestFIB(Interest interest)
{
  int temp;
  //��FIB��δ��
  if(FIBCounter<ForwardingNumber)
  {	
    //FIB�洢Interest�������	
    ForwardingTable[FIBCounter].interestName.DestinationID=interest.interestName.DestinationID;	
    ForwardingTable[FIBCounter].interestName.DataFlag=interest.interestName.DataFlag;	
    ForwardingTable[FIBCounter].interestName.PackageCounter=interest.interestName.PackageCounter;	
    ForwardingTable[FIBCounter].time=0;
    //time��Ҫ��ʱ�������£���������		
    FIBCounter++;
  }
  //��FIB���Ѿ�װ������Ҫʹ���滻���Խ����滻��������̭����ȴ�ʱ�����Ŀ
  else
  {	
    temp=0;		
    for(temp_i=1;temp_i<FIBCounter;++temp_i)	
    {		
      if(ForwardingTable[temp].time<ForwardingTable[temp_i].time && ForwardingTable[temp].used==0 && ForwardingTable[temp_i].used==0)	
        temp=temp_i;	
    }		
    //�滻temp����λ�õ�����		
    //PIT�洢Interest�������
    ForwardingTable[temp].interestName.DestinationID=interest.interestName.DestinationID;	
    ForwardingTable[temp].interestName.DataFlag=interest.interestName.DataFlag;	
    ForwardingTable[temp].interestName.PackageCounter=interest.interestName.PackageCounter;	
    ForwardingTable[temp].time=0;
  }
}


//FIB�������µ�interest���յ���ACK������outcomingQueue���,
//��û�н��ܵ���һ���Ļظ������µ�interest����������OutcomingNumber���ж�
void InsertQueueFIB(uint8 interestID,uint16 nodeid)
{
  //FIB���ж�Ӧ��interestID������ڵ�ŵ����
  //�ж��Ƿ����ظ����ڣ������򲻽������
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


//FIB�����ɾ����ɾ����ǰû�б��ظ�ʹ����ʱ�������Ŀ
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

//FIB����ɾ���ض���Item
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