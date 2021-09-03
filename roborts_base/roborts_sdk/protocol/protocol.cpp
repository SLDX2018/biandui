/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "protocol.h"
#include <iomanip>

namespace roborts_sdk {

Protocol::Protocol(std::shared_ptr<SerialDevice> serial_device_ptr) :     //智能指针，模板类实现
    running_(false),    //构造函数参数列表
    serial_device_ptr_(serial_device_ptr), seq_num_(0),
    is_large_data_protocol_(true), reuse_buffer_(true),
    poll_tick_(10) {

}

Protocol::~Protocol() {
  running_ = false;
  if (send_poll_thread_.joinable()) {
    send_poll_thread_.join();
  }
  if (receive_pool_thread_.joinable()) {
    receive_pool_thread_.join();
  }
  if (recv_stream_ptr_) {
    delete[] recv_stream_ptr_->recv_buff;
    delete recv_stream_ptr_;
  }
  if (recv_buff_ptr_) {
    delete[]recv_buff_ptr_;
  }
  if (recv_container_ptr_) {
    delete recv_container_ptr_;
  }
}

bool Protocol::Init() {

  seq_num_ = 0;
  auto max_buffer_size = BUFFER_SIZE;
  auto max_pack_size = MAX_PACK_SIZE;
  auto session_table_num = SESSION_TABLE_NUM;
  memory_pool_ptr_ = std::make_shared<MemoryPool>(max_pack_size,
                                                  max_buffer_size,
                                                  session_table_num);
  memory_pool_ptr_->Init();

  recv_buff_ptr_ = new uint8_t[BUFFER_SIZE];

  recv_stream_ptr_ = new RecvStream();
  recv_stream_ptr_->recv_buff = new uint8_t[MAX_PACK_SIZE];
  recv_stream_ptr_->recv_index = 0;
  recv_stream_ptr_->reuse_index = 0;
  recv_stream_ptr_->reuse_count = 0;

  recv_container_ptr_ = new RecvContainer();

  SetupSession();   //装载会话，相当于对会话的初始化

  running_ = true;
  send_poll_thread_ = std::thread(&Protocol::AutoRepeatSendCheck, this);  //新线程将在创建新对象后立即开始，并且将与已启动的线程并行执行传递的回调。
  receive_pool_thread_ = std::thread(&Protocol::ReceivePool, this);
  return true;
}

void Protocol::AutoRepeatSendCheck() {    //在上面init函数中开了一个线程，自动检查是否需要重发
  while (running_) {
    unsigned int i;

    std::chrono::steady_clock::time_point current_time_stamp;

    for (i = 1; i < SESSION_TABLE_NUM; i++) { //遍历会话表

      if (cmd_session_table_[i].usage_flag == 1) {  //先判断这个会话有没有使能
        current_time_stamp = std::chrono::steady_clock::now();
        if ((std::chrono::duration_cast<std::chrono::milliseconds>  //函数模板，将两次发送时间戳的差值转换为milliseconds类型  //微秒
            (current_time_stamp - cmd_session_table_[i].pre_time_stamp) >
            cmd_session_table_[i].ack_timeout)) { //超过响应时间

          memory_pool_ptr_->LockMemory();
          if (cmd_session_table_[i].retry_time > 0) {   //重试次数的阈值，大于0即可以重发

            if (cmd_session_table_[i].sent >= cmd_session_table_[i].retry_time) { //重试次数超过阈值
              LOG_ERROR << "Sending timeout, Free session "
                        << static_cast<int>(cmd_session_table_[i].session_id);
              FreeCMDSession(&cmd_session_table_[i]); //失能当前会话，释放内存，usage_flag清0
            } else {
              LOG_ERROR << "Retry session "
                        << static_cast<int>(cmd_session_table_[i].session_id);
              DeviceSend(cmd_session_table_[i].memory_block_ptr->memory_ptr); //串口发送数据
              cmd_session_table_[i].pre_time_stamp = current_time_stamp;
              cmd_session_table_[i].sent++;
            }
          } else {  //重试次数的阈值，为0，则认为是只发送一次
            DLOG_ERROR << "Send once " << i;
            DeviceSend(cmd_session_table_[i].memory_block_ptr->memory_ptr);
            cmd_session_table_[i].pre_time_stamp = current_time_stamp;
          }
          memory_pool_ptr_->UnlockMemory();
        } else {
//        DLOG_INFO<<"Wait for timeout Session: "<< i;
        }
      }
    }
    usleep(1000);
  }
}

// 同一个命令码（cmd_set和cmd_id两个字节）对应的消息（RecvContainer）都放到一个环形缓冲中 CircularBuffer<RecvContainer> 
// 命令码和环形缓冲使用 std::map 容器来对应
void Protocol::ReceivePool() {
  std::chrono::steady_clock::time_point start_time, end_time;
  std::chrono::microseconds execution_duration;
  std::chrono::microseconds cycle_duration = std::chrono::microseconds(int(1e6/READING_RATE));  //1e6微秒=1秒 
  while (running_) {
    start_time = std::chrono::steady_clock::now();    //获取系统时间，从系统启动的时刻开始算，单位纳秒
    RecvContainer *container_ptr = Receive();   //接受数据并在做校验之后，处理数据，在最顶层 //**这个是重点******************
    if (container_ptr) {
      std::lock_guard<std::mutex> lock(mutex_); //std::mutex是互斥锁对象，lock_guard是互斥类，只有构造函数和析构函数，构造函数上锁，析构函数解锁，简化上锁操作
      if (buffer_pool_map_.count(std::make_pair(container_ptr->command_info.cmd_set,
                                                container_ptr->command_info.cmd_id)) == 0) {    //map的count方法，返回值只可能是0或1，map是红黑树，值唯一
        buffer_pool_map_[std::make_pair(container_ptr->command_info.cmd_set,
                                        container_ptr->command_info.cmd_id)]
            = std::make_shared<CircularBuffer<RecvContainer>>(100);   //直接赋值就可以把元素插入map容器，这里只是向map容器中放了一个空的RecvContainer对象

        // std::cout<<"Capture command: "
        //          <<"cmd set: 0x"<< std::setw(2) << std::hex << std::setfill('0') << int(container_ptr->command_info.cmd_set)
        //          <<", cmd id: 0x"<< std::setw(2) << std::hex << std::setfill('0') << int(container_ptr->command_info.cmd_id)
        //          <<", sender: 0x"<< std::setw(2) << std::hex << std::setfill('0') << int(container_ptr->command_info.sender)
        //          <<", receiver: 0x" <<std::setw(2) << std::hex << std::setfill('0') << int(container_ptr->command_info.receiver);

      }
      //1 time copy
      buffer_pool_map_[std::make_pair(container_ptr->command_info.cmd_set,
                                      container_ptr->command_info.cmd_id)]->Push(*container_ptr);   //向相应的环形缓冲中push一个RecvContainer对象
    }
    end_time = std::chrono::steady_clock::now();    //计算数据接受用了多长时间，用来保证接受频率
    std::chrono::microseconds execution_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    if (cycle_duration > execution_duration){
      std::this_thread::sleep_for(cycle_duration - execution_duration);   //如果提前完成任务之后延时，保证接受数据的频率是 READING_RATE
    }

  }
}

//从接收池中取出一个消息，这是个接口，ReceivePool对应的线程一直在运行，数据在环形缓冲中不断更新，但是调用Take才能取出一个数据
bool Protocol::Take(const CommandInfo *command_info,
                    MessageHeader *message_header,
                    void *message_data) {

  std::lock_guard<std::mutex> lock(mutex_);
  if (buffer_pool_map_.count(std::make_pair(command_info->cmd_set,
                                            command_info->cmd_id)) == 0) {
//    DLOG_ERROR<<"take failed";
    return false;
  } else {        // 通过命令码找到该命令码对应的环形缓冲
    //1 time copy
    RecvContainer container;

    if (!buffer_pool_map_[std::make_pair(command_info->cmd_set,
                                         command_info->cmd_id)]->Pop(container)) {    //Pop传入参数是一个RecvContainer类型的引用，从消息组中Pop一个消息
//      DLOG_EVERY_N(ERROR, 100)<<"nothing to take";
      return false;
    }

    // 再次匹配一下要接受的消息和实际接受的消息是不是能对应上，主要看一些固定的字段？？？   //********************
    bool mismatch = false;

    if (int(container.command_info.need_ack) != int(command_info->need_ack)){
      DLOG_ERROR << "Requested need_ack: "<< int(command_info->need_ack)
                 << ", Get need_ack: "<< int(container.command_info.need_ack);
      mismatch = true;
    }

    if (container.message_header.is_ack){   // 如果需要ack，那container的接收地址应该是当前消息的发送地址？？？？   //********************
      if (int(container.command_info.receiver) != int(command_info->sender)){
        DLOG_ERROR << "Requested ACK receiver: "<< std::setw(2) << std::hex << std::setfill('0') << int(command_info->sender)
                   << ", Get ACK receiver: "<< std::setw(2) << std::hex << std::setfill('0') << int(container.command_info.receiver);
        mismatch = true;
      }
      if (int(container.command_info.sender) != int(command_info->receiver)){
        DLOG_ERROR << "Requested ACK sender: "<< std::setw(2) << std::hex << std::setfill('0') << int(command_info->receiver)
                   << ", Get ACK sender: "<< std::setw(2) << std::hex << std::setfill('0') << int(container.command_info.sender);
        mismatch = true;
      }
    }
    else{
      if (int(container.command_info.receiver) != int(command_info->receiver)){
        DLOG_ERROR << "Requested receiver: "<< std::setw(2) << std::hex << std::setfill('0') << int(container.command_info.receiver)
                   << ", Get receiver: "<< std::setw(2) << std::hex << std::setfill('0') << int(container.command_info.receiver);
        mismatch = true;
      }

      if (int(container.command_info.sender) != int(command_info->sender)){
        DLOG_ERROR << "Requested sender: "<< std::setw(2) << std::hex << std::setfill('0') << int(command_info->sender)
                   << ", Get sender: "<< std::setw(2) << std::hex << std::setfill('0') << int(container.command_info.sender);
        mismatch = true;
      }
    }

    if (int(container.command_info.length) !=int(command_info->length)){
      DLOG_ERROR << "Requested length: "<< int(command_info->length)
                 <<", Get length: "<< int(container.command_info.length);
      mismatch = true;
    }

    if(mismatch){   //没有匹配上
      buffer_pool_map_[std::make_pair(command_info->cmd_set,
                                      command_info->cmd_id)]->Push(container);  //那为什么没有匹配上还要重新push到环形缓冲中？？ //********************
      return false;
    }

    //1 time copy
    memcpy(message_header, &(container.message_header), sizeof(message_header));  //操作指针，将数据从函数内传出
    memcpy(message_data, &(container.message_data), command_info->length);

    return true;
  }
}
// SendResponse在分发层有方法调用了，但是实际代码中没有使用，猜测是pc作为“主设备”，和底板通信要么请求数据，要么直接发数据，底板没有向pc请求数据的需求
bool Protocol::SendResponse(const CommandInfo *command_info,
                            const MessageHeader *message_header,
                            void *message_data) {
  return SendACK(message_header->session_id,
                 message_header->seq_num,
                 command_info->receiver,
                 message_data, command_info->length);
}
bool Protocol::SendRequest(const CommandInfo *command_info,   //command_info是要发送出去的，这个对象里面有 need_ack 来确定是否需要ack，和SendMessage区分
                           MessageHeader *message_header,   //接收到的消息头
                           void *message_data) {            //接收到的消息体
  return SendCMD(command_info->cmd_set, command_info->cmd_id,
                 command_info->receiver, message_data, command_info->length,
                 CMDSessionMode::CMD_SESSION_AUTO, message_header);
}
bool Protocol::SendMessage(const CommandInfo *command_info,   //直接发送数据，不需要ack
                           void *message_data) {
  return SendCMD(command_info->cmd_set, command_info->cmd_id,
                 command_info->receiver, message_data, command_info->length,
                 CMDSessionMode::CMD_SESSION_0);
}

/*************************** Session Management **************************/
void Protocol::SetupSession() {
  uint8_t i, j;
  for (i = 0; i < SESSION_TABLE_NUM; i++) {
    cmd_session_table_[i].session_id = i;
    cmd_session_table_[i].usage_flag = false;
    cmd_session_table_[i].memory_block_ptr = nullptr;
  }

  for (i = 0; i < RECEIVER_NUM; i++) {
    for (j = 0; j < (SESSION_TABLE_NUM - 1); j++) {
      ack_session_table_[i][j].session_id = j + 1;
      ack_session_table_[i][j].session_status = ACKSessionStatus::ACK_SESSION_IDLE;
      ack_session_table_[i][j].memory_block_ptr = nullptr;
    }
  }
}

CMDSession *Protocol::AllocCMDSession(CMDSessionMode session_mode, uint16_t size) {   //usage_flag==0时，即没有占用的时候才能申请内存
  uint32_t i;   
  MemoryBlock *memory_block_ptr = nullptr;

  if (session_mode == CMDSessionMode::CMD_SESSION_0 || session_mode == CMDSessionMode::CMD_SESSION_1) {
    if (cmd_session_table_[(uint16_t) session_mode].usage_flag == 0) {
      i = static_cast<uint32_t>(session_mode);    //良性类型转换，一般不错 http://c.biancheng.net/cpp/biancheng/view/3297.html
    } else {
      DLOG_ERROR << "session " << static_cast<uint32_t>(session_mode) << " is busy\n";
      return nullptr;
    }
  } else {
    for (i = 2; i < SESSION_TABLE_NUM; i++) {   //找到第一个空闲的session
      if (cmd_session_table_[i].usage_flag == 0) {
        break;
      }
    }

  }

  if (i < 32 && cmd_session_table_[i].usage_flag == 0) {

    cmd_session_table_[i].usage_flag = 1;
    memory_block_ptr = memory_pool_ptr_->AllocMemory(size);   //申请内存
    if (memory_block_ptr == nullptr) {
      cmd_session_table_[i].usage_flag = 0;
    } else {
//      DLOG_INFO<<"find "<<i;
      cmd_session_table_[i].memory_block_ptr = memory_block_ptr;
      return &cmd_session_table_[i];
    }
  } else {
    DLOG_INFO << "All usable CMD session id are occupied";
  }

  return nullptr;
}

void Protocol::FreeCMDSession(CMDSession *session_ptr) {
  if (session_ptr->usage_flag == 1) {
    memory_pool_ptr_->FreeMemory(session_ptr->memory_block_ptr);
    session_ptr->usage_flag = 0;
  }
}

ACKSession *Protocol::AllocACKSession(uint8_t receiver, uint16_t session_id, uint16_t size) {
  MemoryBlock *memory_block_ptr = nullptr;
  if (session_id > 0 && session_id < 32) {
    if (ack_session_table_[receiver][session_id - 1].memory_block_ptr) {
      FreeACKSession(&ack_session_table_[receiver][session_id - 1]);
    }

    memory_block_ptr = memory_pool_ptr_->AllocMemory(size);
    if (memory_block_ptr == nullptr) {
      DLOG_ERROR << "there is not enough memory";
      return nullptr;
    } else {
      ack_session_table_[receiver][session_id - 1].memory_block_ptr = memory_block_ptr;
      return &ack_session_table_[receiver][session_id - 1];
    }
  }
  return nullptr;
}

void Protocol::FreeACKSession(ACKSession *session_ptr) {
  memory_pool_ptr_->FreeMemory(session_ptr->memory_block_ptr);
}

/****************************** Send Pipline *****************************/
bool Protocol::SendCMD(uint8_t cmd_set, uint8_t cmd_id, uint8_t receiver,
                       void *data_ptr, uint16_t data_length,
                       CMDSessionMode session_mode, MessageHeader* message_header,
                       std::chrono::milliseconds ack_timeout, int retry_time) {

  CMDSession *cmd_session_ptr = nullptr;
  Header *header_ptr = nullptr;
  uint8_t cmd_set_prefix[] = {cmd_id, cmd_set};
  uint32_t crc_data;

  uint16_t pack_length = 0;


  //calculate pack_length first
  if (data_length == 0 || data_ptr == nullptr) {
    DLOG_ERROR << "No data send.";
    return false;
  }
  pack_length = HEADER_LEN +
      CMD_SET_PREFIX_LEN +
      data_length + CRC_DATA_LEN;

  //second get the param into the session
  switch (session_mode) {

    case CMDSessionMode::CMD_SESSION_0:
      //lock
      memory_pool_ptr_->LockMemory();
      cmd_session_ptr = AllocCMDSession(CMDSessionMode::CMD_SESSION_0, pack_length);

      if (cmd_session_ptr == nullptr) {
        //unlock
        memory_pool_ptr_->UnlockMemory();
        DLOG_ERROR << "Allocate CMD session failed.";
        return false;
      }

      //pack into cmd_session memory_block
      header_ptr = (Header *) cmd_session_ptr->memory_block_ptr->memory_ptr;
      header_ptr->sof = SOF;
      header_ptr->length = pack_length;
      header_ptr->version = VERSION;
      header_ptr->session_id = cmd_session_ptr->session_id;
      header_ptr->is_ack = 0;
      header_ptr->reserved0 = 0;
      header_ptr->sender = DEVICE;
//      header_ptr->sender = 0x01;    //chassis_addr test
      header_ptr->receiver = receiver;
      header_ptr->reserved1 = 0;
      header_ptr->seq_num = seq_num_;
      header_ptr->crc = CRC16Calc(cmd_session_ptr->memory_block_ptr->memory_ptr, HEADER_LEN - CRC_HEAD_LEN);

      if(message_header){
        message_header->is_ack = false;
        message_header->seq_num = seq_num_;
        message_header->session_id = cmd_session_ptr->session_id;
      }

      // pack the cmd prefix ,data and data crc into memory block one by one
      memcpy(cmd_session_ptr->memory_block_ptr->memory_ptr + HEADER_LEN, cmd_set_prefix, CMD_SET_PREFIX_LEN);
      memcpy(cmd_session_ptr->memory_block_ptr->memory_ptr + HEADER_LEN + CMD_SET_PREFIX_LEN, data_ptr, data_length);

      crc_data = CRC32Calc(cmd_session_ptr->memory_block_ptr->memory_ptr, pack_length - CRC_DATA_LEN);
      memcpy(cmd_session_ptr->memory_block_ptr->memory_ptr + pack_length - CRC_DATA_LEN, &crc_data, CRC_DATA_LEN);

      // send it using device
      DeviceSend(cmd_session_ptr->memory_block_ptr->memory_ptr);

      seq_num_++;
      FreeCMDSession(cmd_session_ptr);
      //unlock
      memory_pool_ptr_->UnlockMemory();
      break;

    case CMDSessionMode::CMD_SESSION_1:
      //lock
      memory_pool_ptr_->LockMemory();
      cmd_session_ptr = AllocCMDSession(CMDSessionMode::CMD_SESSION_1, pack_length);

      if (cmd_session_ptr == nullptr) {
        //unlock
        memory_pool_ptr_->UnlockMemory();
        DLOG_ERROR << "Allocate CMD session failed.";
        return false;
      }

      //may be used more than once, seq_num_ should increase if duplicated.
      if (seq_num_ == cmd_session_ptr->pre_seq_num) {
        seq_num_++;
      }

      //pack into cmd_session memory_block
      header_ptr = (Header *) cmd_session_ptr->memory_block_ptr->memory_ptr;
      header_ptr->sof = SOF;
      header_ptr->length = pack_length;
      header_ptr->version = VERSION;
      header_ptr->session_id = cmd_session_ptr->session_id;
      header_ptr->is_ack = 0;
      header_ptr->reserved0 = 0;
      header_ptr->sender = DEVICE;
      header_ptr->receiver = receiver;
      header_ptr->reserved1 = 0;
      header_ptr->seq_num = seq_num_;
      header_ptr->crc = CRC16Calc(cmd_session_ptr->memory_block_ptr->memory_ptr, HEADER_LEN - CRC_HEAD_LEN);

      if(message_header){
        message_header->is_ack = false;
        message_header->seq_num = seq_num_;
        message_header->session_id = cmd_session_ptr->session_id;
      }

      // pack the cmd prefix ,data and data crc into memory block one by one
      memcpy(cmd_session_ptr->memory_block_ptr->memory_ptr + HEADER_LEN, cmd_set_prefix, CMD_SET_PREFIX_LEN);
      memcpy(cmd_session_ptr->memory_block_ptr->memory_ptr + HEADER_LEN + CMD_SET_PREFIX_LEN, data_ptr, data_length);

      crc_data = CRC32Calc(cmd_session_ptr->memory_block_ptr->memory_ptr, pack_length - CRC_DATA_LEN);
      memcpy(cmd_session_ptr->memory_block_ptr->memory_ptr + pack_length - CRC_DATA_LEN, &crc_data, CRC_DATA_LEN);

      // seem useless
      cmd_session_ptr->cmd_id = cmd_id;
      cmd_session_ptr->cmd_set = cmd_set;
      cmd_session_ptr->pre_seq_num = seq_num_++;

      cmd_session_ptr->ack_timeout = (ack_timeout > poll_tick_) ? ack_timeout : poll_tick_;
      cmd_session_ptr->pre_time_stamp = std::chrono::steady_clock::now();
      cmd_session_ptr->sent = 1;
      cmd_session_ptr->retry_time = 1;
      // send it using device
      DeviceSend(cmd_session_ptr->memory_block_ptr->memory_ptr);
      //unlock
      memory_pool_ptr_->UnlockMemory();
      break;

    case CMDSessionMode::CMD_SESSION_AUTO:
      //lock
      memory_pool_ptr_->LockMemory();
      cmd_session_ptr = AllocCMDSession(CMDSessionMode::CMD_SESSION_AUTO, pack_length);

      if (cmd_session_ptr == nullptr) {
        //unlock
        memory_pool_ptr_->UnlockMemory();
        DLOG_ERROR << "Allocate CMD session failed.";
        return false;
      }

      //may be used more than once, seq_num_ should increase if duplicated.
      if (seq_num_ == cmd_session_ptr->pre_seq_num) {
        seq_num_++;
      }

      //pack into cmd_session memory_block
      header_ptr = (Header *) cmd_session_ptr->memory_block_ptr->memory_ptr;
      header_ptr->sof = SOF;
      header_ptr->length = pack_length;
      header_ptr->version = VERSION;
      header_ptr->session_id = cmd_session_ptr->session_id;
      header_ptr->is_ack = 0;
      header_ptr->reserved0 = 0;
      header_ptr->sender = DEVICE;
      header_ptr->receiver = receiver;
      header_ptr->reserved1 = 0;
      header_ptr->seq_num = seq_num_;
      header_ptr->crc = CRC16Calc(cmd_session_ptr->memory_block_ptr->memory_ptr, HEADER_LEN - CRC_HEAD_LEN);


      if(message_header){
        message_header->is_ack = false;
        message_header->seq_num = seq_num_;
        message_header->session_id = cmd_session_ptr->session_id;
      }

      // pack the cmd prefix ,data and data crc into memory block one by one
      memcpy(cmd_session_ptr->memory_block_ptr->memory_ptr + HEADER_LEN, cmd_set_prefix, CMD_SET_PREFIX_LEN);
      memcpy(cmd_session_ptr->memory_block_ptr->memory_ptr + HEADER_LEN + CMD_SET_PREFIX_LEN, data_ptr, data_length);

      crc_data = CRC32Calc(cmd_session_ptr->memory_block_ptr->memory_ptr, pack_length - CRC_DATA_LEN);
      memcpy(cmd_session_ptr->memory_block_ptr->memory_ptr + pack_length - CRC_DATA_LEN, &crc_data, CRC_DATA_LEN);

      // seem useless
      cmd_session_ptr->cmd_id = cmd_id;
      cmd_session_ptr->cmd_set = cmd_set;
      cmd_session_ptr->pre_seq_num = seq_num_++;

      cmd_session_ptr->ack_timeout = (ack_timeout > poll_tick_) ? ack_timeout : poll_tick_;
      cmd_session_ptr->pre_time_stamp = std::chrono::steady_clock::now();
      cmd_session_ptr->sent = 1;
      cmd_session_ptr->retry_time = retry_time;
      // send it using device
      DeviceSend(cmd_session_ptr->memory_block_ptr->memory_ptr);
      //unlock
      memory_pool_ptr_->UnlockMemory();
      break;

    default:DLOG_ERROR << "session mode is not valid";
      return false;
  }

  return true;

}

bool Protocol::SendACK(uint8_t session_id, uint16_t seq_num, uint8_t receiver,
                       void *ack_ptr, uint16_t ack_length) {
  ACKSession *ack_session_ptr = nullptr;
  Header *header_ptr = nullptr;
  uint32_t crc_data = 0;
  uint16_t pack_length = 0;

  if (ack_ptr == nullptr || ack_length == 0) {
    pack_length = HEADER_LEN;
  } else {
    pack_length = HEADER_LEN +
        ack_length + CRC_DATA_LEN;
  }

  if (session_id == 0 || session_id > 31) {
    DLOG_ERROR << ("ack session id should be from 1 to 31.");
    return false;
  } else {

    //lock
    memory_pool_ptr_->LockMemory();
    ack_session_ptr = AllocACKSession(receiver, session_id, pack_length);

    if (ack_session_ptr == nullptr) {
      //unlock
      memory_pool_ptr_->UnlockMemory();
      DLOG_ERROR << "Allocate ACK session failed.";
      return false;
    }

    //pack into ack_session memory_block
    header_ptr = (Header *) ack_session_ptr->memory_block_ptr->memory_ptr;
    header_ptr->sof = SOF;
    header_ptr->length = pack_length;
    header_ptr->version = VERSION;
    header_ptr->session_id = ack_session_ptr->session_id;
    header_ptr->is_ack = 1;
    header_ptr->reserved0 = 0;
    header_ptr->sender = DEVICE;
    header_ptr->receiver = receiver;
    header_ptr->reserved1 = 0;
    header_ptr->seq_num = seq_num;
    header_ptr->crc = CRC16Calc(ack_session_ptr->memory_block_ptr->memory_ptr, HEADER_LEN - CRC_HEAD_LEN);

    // pack the cmd prefix ,data and data crc into memory block one by one
    if (ack_ptr != nullptr && ack_length != 0) {
      memcpy(ack_session_ptr->memory_block_ptr->memory_ptr + HEADER_LEN, ack_ptr, ack_length);
      crc_data = CRC32Calc(ack_session_ptr->memory_block_ptr->memory_ptr, pack_length - CRC_DATA_LEN);
      memcpy(ack_session_ptr->memory_block_ptr->memory_ptr + pack_length - CRC_DATA_LEN, &crc_data, CRC_DATA_LEN);
    }

    // send it using device
    DeviceSend(ack_session_ptr->memory_block_ptr->memory_ptr);
    ack_session_table_[receiver][session_id - 1].session_status = ACKSessionStatus::ACK_SESSION_USING;
    //unlock
    memory_pool_ptr_->UnlockMemory();
    return true;
  }

}

bool Protocol::DeviceSend(uint8_t *buf) {
  int ans;
  Header *header_ptr = (Header *) buf;

// For debug and visualzation:
// ans = header_ptr->length;
//  for(int i =0;i<header_ptr->length;i++){
//    printf("send_byte %d:\t %X\n ", i, buf[i]);
//  }
//  std::cout<<"----------------"<<std::endl;
  ans = serial_device_ptr_->Write(buf, header_ptr->length);

  if (ans <= 0) {
    DLOG_ERROR << "Port failed.";
  } else if (ans != header_ptr->length) {
    DLOG_ERROR << "Port send failed, send length:" << ans << "package length" << header_ptr->length;
  } else {
    DLOG_INFO << "Port send success with length: " << header_ptr->length;
    return true;
  }
  return false;
}

/****************************** Recv Pipline ******************************/
RecvContainer *Protocol::Receive() {  //从硬件读到recv_buff_ptr_

  //! Bool to check if the protocol parser has finished a full frame
  bool is_frame = false;

  //! Step 1: Check if the buffer has been consumed   //检查是否读完数据，并重置pos和len，开始下一次读取数据
  if (recv_buff_read_pos_ >= recv_buff_read_len_) {   //上一次读完的时候，for遍历最后，recv_buff_read_pos_ 比 recv_buff_read_len_ 大
    recv_buff_read_pos_ = 0;
    recv_buff_read_len_ = serial_device_ptr_->Read(recv_buff_ptr_, BUFFER_SIZE);  //Read返回值为读到的字节数    
  }

  //! Step 2:
  //! For large data protocol, store the value and only verify the header
  //! For small data protocol, Go through the buffer and return when you
  //! see a full frame. buf_read_pos will maintain state about how much
  //! buffer data we have already read

  //TODO: unhandled
  if (is_large_data_protocol_ && recv_buff_read_len_ == BUFFER_SIZE) {    //是否启用了大数据包的读取（比buffer大的数据包）

    memcpy(recv_stream_ptr_->recv_buff + (recv_stream_ptr_->recv_index), recv_buff_ptr_,    //这种情况下recv_stream_ptr_是循环使用的
           BUFFER_SIZE);
    recv_stream_ptr_->recv_index += BUFFER_SIZE;
    recv_buff_read_pos_ = BUFFER_SIZE;                  //********************
  } else {
    for (recv_buff_read_pos_; recv_buff_read_pos_ < recv_buff_read_len_;    //0~从硬件中读取的字节数
         recv_buff_read_pos_++) {
      is_frame = ByteHandler(recv_buff_ptr_[recv_buff_read_pos_]);

      if (is_frame) {
        return recv_container_ptr_;
      }
    }
  }

  //! Step 3: If we don't find a full frame by this time, return nullptr.
  return nullptr;
}

bool Protocol::ByteHandler(const uint8_t byte) {      //字节搬运  //从buffer到 stream
  recv_stream_ptr_->reuse_count = 0;
  recv_stream_ptr_->reuse_index = MAX_PACK_SIZE;

  bool is_frame = StreamHandler(byte);

  if (reuse_buffer_) {    //true
    if (recv_stream_ptr_->reuse_count != 0) {

      while (recv_stream_ptr_->reuse_index < MAX_PACK_SIZE) {   // 遍历recv_stream_ptr_->recv_buff[]
        /*! @note because reuse_index maybe re-located, so reuse_index must
         *  be
         *  always point to un-used index
         *  re-loop the buffered data
         *  */
        is_frame = StreamHandler(recv_stream_ptr_->recv_buff[recv_stream_ptr_->reuse_index++]);
      }
      recv_stream_ptr_->reuse_count = 0;
    }
  }
  return is_frame;
}

bool Protocol::StreamHandler(uint8_t byte) {    //保存一个字节到recv_stream_ptr_->recv_buff //从硬件读出的数据进recv_buff_ptr_，再搬运到stream

  // push the byte into filter buffer
  if (recv_stream_ptr_->recv_index < MAX_PACK_SIZE) {
    recv_stream_ptr_->recv_buff[recv_stream_ptr_->recv_index] = byte;
    recv_stream_ptr_->recv_index++;
  } else {
    LOG_ERROR << "Buffer overflow";   //溢出
    memset(recv_stream_ptr_->recv_buff, 0, recv_stream_ptr_->recv_index);
    recv_stream_ptr_->recv_index = 0;
  }

  bool is_frame = CheckStream();
  return is_frame;
}

bool Protocol::CheckStream() {         //检查是不是一个数据包的开始，找到完整数据包
  Header *header_ptr = (Header *) (recv_stream_ptr_->recv_buff);

  // std::cout << "header_sof" << header_ptr->sof  <<  std::endl
  //           <<  "header_receiver" << header_ptr->receiver;


  bool is_frame = false;
  if (recv_stream_ptr_->recv_index < HEADER_LEN) {  //帧头长度
    return false;
  } else if (recv_stream_ptr_->recv_index == HEADER_LEN) {  //帧头校验
    is_frame = VerifyHeader();
  // std::cout << "file:" << __FILE__ << ":"<< __LINE__ << "\r\n";
  } else if (recv_stream_ptr_->recv_index == header_ptr->length) {  //数据域校验
  // std::cout << "file:" << __FILE__ << ":"<< __LINE__ << "\r\n";
    is_frame = VerifyData();
  }

  return is_frame;
}

bool Protocol::VerifyHeader() {   //检查帧头
  Header *header_ptr = (Header *) (recv_stream_ptr_->recv_buff);
  bool is_frame = false;

  if ((header_ptr->sof == SOF) && (header_ptr->version == VERSION) &&
      (header_ptr->length < MAX_PACK_SIZE) && (header_ptr->reserved0 == 0) &&
      (header_ptr->reserved1 == 0) && (header_ptr->receiver == DEVICE || header_ptr->receiver == 0xFF) &&
      CRCHeadCheck((uint8_t *) header_ptr, HEADER_LEN)) {   //帧头数据正确CRC通过
    // It is an unused part because minimum package is at least longer than a header
    if (header_ptr->length == HEADER_LEN) {   //

      is_frame = ContainerHandler();    //容器搬运？？********************
      //prepare data stream
      PrepareStream();    //移动数据，去掉 header 部分
    }

  } else {
    //shift the data stream
    ShiftStream();    //移动数据流，解决粘包问题，找到帧头，类似类似滑动窗口
  }

  return is_frame;

}

bool Protocol::VerifyData() {
  Header *header_ptr = (Header *) (recv_stream_ptr_->recv_buff);
  bool is_frame = false;

// if(header_ptr->sof == SOF)
// {
//   std::cout << header_ptr->sof << "-"
//             << header_ptr->length << "-"
//             << header_ptr->reserved0 << "-"
//             << header_ptr->reserved1 << "-"
//             << header_ptr->sender << "-"
//             << header_ptr->receiver << "-"
//             << header_ptr->seq_num << "-"
//             << std::endl;
// }

  if (CRCTailCheck((uint8_t *) header_ptr, header_ptr->length)) {

    is_frame = ContainerHandler();
    //prepare data stream
    PrepareStream();
  } else {      // 能到这里说明，帧头校验过了，数据部分的长度和帧头中的length对上了，但是数据部分的crc检验没通过
    //reuse the data stream
    ReuseStream();
  }

  return is_frame;
}

bool Protocol::ContainerHandler() {   //到这已经可以保证是完整数据包了，通过了帧头检验，开始处理数据内容  //从stream到container

  // std::cout << "file:" << __FILE__ << ":"<< __LINE__ << "\r\n";
  Header *session_header_ptr = nullptr;   //通信的帧头
  Header *header_ptr = (Header *) (recv_stream_ptr_->recv_buff);
  bool is_frame = false;

  if (header_ptr->is_ack) {   //需要ack 

    if (header_ptr->session_id > 0 && header_ptr->session_id < 32) {    //会话id合法，保证cmd_session_table_访问不会越界 ********************
      if (cmd_session_table_[header_ptr->session_id].usage_flag == 1) {

        memory_pool_ptr_->LockMemory();
        session_header_ptr = (Header *) cmd_session_table_[header_ptr->session_id].memory_block_ptr->memory_ptr;  //每个会话有一个内存块，但是这个好像没有申请内存？alloc？********************

        if (session_header_ptr->session_id == header_ptr->session_id  // 在 SetupSession 的时候已经对session_id赋值了，0-31
          // HotFix: Commented here. Redefine that ack and cmd can have different seq num during communication
          // && session_header_ptr->seq_num == header_ptr->seq_num
            ) {

          recv_container_ptr_->message_header.is_ack = true;
          recv_container_ptr_->message_header.seq_num = header_ptr->seq_num;//HotFix as above: session_header_ptr->seq_num originally

          recv_container_ptr_->message_header.session_id = header_ptr->session_id;
          recv_container_ptr_->command_info.length = header_ptr->length - HEADER_LEN - CRC_DATA_LEN;
          recv_container_ptr_->command_info.sender = header_ptr->sender;
          recv_container_ptr_->command_info.receiver = header_ptr->receiver;
          recv_container_ptr_->command_info.cmd_set = cmd_session_table_[header_ptr->session_id].cmd_set; //命令码部分，共2字节   //并没有找到在哪里赋值的********************
          recv_container_ptr_->command_info.cmd_id = cmd_session_table_[header_ptr->session_id].cmd_id;   // ********************
          recv_container_ptr_->command_info.need_ack = true;

          memcpy(recv_container_ptr_->message_data.raw_data, (uint8_t *) header_ptr + HEADER_LEN,   //拷贝通信数据包中的数据部分
                 header_ptr->length - HEADER_LEN - CRC_DATA_LEN);

          is_frame = true;
          FreeCMDSession(&cmd_session_table_[header_ptr->session_id]); // 当前会话完成，失能当前会话，释放内存，usage_flag清0
          memory_pool_ptr_->UnlockMemory();
          // TODO: notify mechanism ,notify ack received and get recv container ready

        } else {
          memory_pool_ptr_->UnlockMemory();
        }
      } //usage_flag == 1
    } //session_id在1-31  合法session_id
  } else {  //不需要发送 ack  
    switch (header_ptr->session_id) {
      case 0:   
        recv_container_ptr_->message_header.is_ack = false;
        recv_container_ptr_->message_header.seq_num = header_ptr->seq_num;
        recv_container_ptr_->message_header.session_id = header_ptr->session_id;
        recv_container_ptr_->command_info.sender = header_ptr->sender;
        recv_container_ptr_->command_info.receiver = header_ptr->receiver;
        recv_container_ptr_->command_info.length = header_ptr->length - HEADER_LEN - CMD_SET_PREFIX_LEN - CRC_DATA_LEN;
        recv_container_ptr_->command_info.cmd_set = *((uint8_t *) header_ptr + HEADER_LEN + 1);   //命令码部分，共2字节 ********************
        recv_container_ptr_->command_info.cmd_id = *((uint8_t *) header_ptr + HEADER_LEN);    // ********************
        recv_container_ptr_->command_info.need_ack = false;

        memcpy(recv_container_ptr_->message_data.raw_data, (uint8_t *) header_ptr + HEADER_LEN + CMD_SET_PREFIX_LEN,
               header_ptr->length - HEADER_LEN - CMD_SET_PREFIX_LEN - CRC_DATA_LEN);

        is_frame = true;
//std::cout << "file:" << __FILE__ << ":"<< __LINE__ << "\r\n";
//std::cout << "cmd:" << recv_container_ptr_->command_info.cmd_set+0 << recv_container_ptr_->command_info.cmd_id+0 << "\r\n";
        break;
      case 1:
        //TODO: Currently regard session_1 as session_auto but never change the ack session status, ack session always stay idle status for session_1
        //break;
      default:
        if (header_ptr->session_id > 31) {
          return false;
        } else {
          //TODO ack session table is supposed to indexed by command sender instead of receiver, as receiver is always local device address
          switch (ack_session_table_[header_ptr->receiver][header_ptr->session_id - 1].session_status) {
            case ACKSessionStatus::ACK_SESSION_IDLE:

              if (header_ptr->session_id > 1) {
                memory_pool_ptr_->LockMemory();
                ack_session_table_[header_ptr->receiver][header_ptr->session_id - 1].session_status =
                    ACKSessionStatus::ACK_SESSION_PROCESS;
                memory_pool_ptr_->UnlockMemory();
              }

              recv_container_ptr_->message_header.is_ack = false;
              recv_container_ptr_->message_header.seq_num = header_ptr->seq_num;
              recv_container_ptr_->message_header.session_id = header_ptr->session_id;
              recv_container_ptr_->command_info.sender = header_ptr->sender;
              recv_container_ptr_->command_info.receiver = header_ptr->receiver;
              recv_container_ptr_->command_info.length =
                  header_ptr->length - HEADER_LEN - CMD_SET_PREFIX_LEN - CRC_DATA_LEN;
              recv_container_ptr_->command_info.cmd_set = *((uint8_t *) header_ptr + HEADER_LEN + 1);
              recv_container_ptr_->command_info.cmd_id = *((uint8_t *) header_ptr + HEADER_LEN);
              recv_container_ptr_->command_info.need_ack = true;

              memcpy(recv_container_ptr_->message_data.raw_data,
                     (uint8_t *) header_ptr + HEADER_LEN + CMD_SET_PREFIX_LEN,
                     header_ptr->length - HEADER_LEN - CMD_SET_PREFIX_LEN - CRC_DATA_LEN);
              is_frame = true;
              break;

            case ACKSessionStatus::ACK_SESSION_PROCESS:

              DLOG_INFO << "Wait for app ack for session " << header_ptr->session_id;
              break;

            case ACKSessionStatus::ACK_SESSION_USING:

              memory_pool_ptr_->LockMemory();
              session_header_ptr = (Header *) ack_session_table_[header_ptr->receiver][header_ptr->session_id
                  - 1].memory_block_ptr->memory_ptr;

              if (session_header_ptr->seq_num == header_ptr->seq_num) {
                DeviceSend((uint8_t *) session_header_ptr);
                memory_pool_ptr_->UnlockMemory();
              } else {

                ack_session_table_[header_ptr->receiver][header_ptr->session_id - 1].session_status =
                    ACKSessionStatus::ACK_SESSION_PROCESS;
                memory_pool_ptr_->UnlockMemory();

                recv_container_ptr_->message_header.is_ack = false;
                recv_container_ptr_->message_header.seq_num = header_ptr->seq_num;
                recv_container_ptr_->message_header.session_id = header_ptr->session_id;
                recv_container_ptr_->command_info.sender = header_ptr->sender;
                recv_container_ptr_->command_info.receiver = header_ptr->receiver;
                recv_container_ptr_->command_info.length =
                    header_ptr->length - HEADER_LEN - CMD_SET_PREFIX_LEN - CRC_DATA_LEN;
                recv_container_ptr_->command_info.cmd_set = *((uint8_t *) header_ptr + HEADER_LEN + 1);
                recv_container_ptr_->command_info.cmd_id = *((uint8_t *) header_ptr + HEADER_LEN);
                recv_container_ptr_->command_info.need_ack = true;
                memcpy(recv_container_ptr_->message_data.raw_data,
                       (uint8_t *) header_ptr + HEADER_LEN + CMD_SET_PREFIX_LEN,
                       header_ptr->length - HEADER_LEN - CMD_SET_PREFIX_LEN - CRC_DATA_LEN);
                is_frame = true;

              }
              break;

            default:
              DLOG_ERROR << "Wrong ACK session status which is out of 0-2";
          }

        }

    }
  }

  return is_frame;
}

/*************************** Stream Management*****************************/
void Protocol::PrepareStream() {
  uint32_t bytes_to_move = HEADER_LEN - 1;
  uint32_t index_of_move = recv_stream_ptr_->recv_index - bytes_to_move;  //这里是减，recv_index指向的是最后一个字节的数据,是索引

  memmove(recv_stream_ptr_->recv_buff, recv_stream_ptr_->recv_buff + index_of_move, bytes_to_move);
  memset(recv_stream_ptr_->recv_buff + bytes_to_move, 0, index_of_move);
  recv_stream_ptr_->recv_index = bytes_to_move;
}

void Protocol::ShiftStream() {
  if (recv_stream_ptr_->recv_index) {
    recv_stream_ptr_->recv_index--;
    if (recv_stream_ptr_->recv_index) {
      memmove(recv_stream_ptr_->recv_buff, recv_stream_ptr_->recv_buff + 1, recv_stream_ptr_->recv_index);  //相当于丢弃recv_buff第0字节，整体前移
    }
  }
}

void Protocol::ReuseStream() {
  uint8_t *buff_ptr = recv_stream_ptr_->recv_buff;
  uint16_t bytes_to_move = recv_stream_ptr_->recv_index - HEADER_LEN;
  uint8_t *src_ptr = buff_ptr + HEADER_LEN;

  uint16_t n_dest_index = recv_stream_ptr_->reuse_index - bytes_to_move;
  uint8_t *dest_ptr = buff_ptr + n_dest_index;

  memmove(dest_ptr, src_ptr, bytes_to_move);

  recv_stream_ptr_->recv_index = HEADER_LEN;
  ShiftStream();

  recv_stream_ptr_->reuse_index = n_dest_index;
  recv_stream_ptr_->reuse_count++;
}

/*************************** CRC Calculationns ****************************/
uint16_t Protocol::CRC16Update(uint16_t crc, uint8_t ch) {
  uint16_t tmp;
  uint16_t msg;

  msg = 0x00ff & static_cast<uint16_t>(ch);
  tmp = crc ^ msg;
  crc = (crc >> 8) ^ crc_tab16[tmp & 0xff];

  return crc;
}

uint32_t Protocol::CRC32Update(uint32_t crc, uint8_t ch) {
  uint32_t tmp;
  uint32_t msg;

  msg = 0x000000ffL & static_cast<uint32_t>(ch);
  tmp = crc ^ msg;
  crc = (crc >> 8) ^ crc_tab32[tmp & 0xff];
  return crc;
}

uint16_t Protocol::CRC16Calc(const uint8_t *data_ptr, size_t length) {
  size_t i;
  uint16_t crc = CRC_INIT;

  for (i = 0; i < length; i++) {
    crc = CRC16Update(crc, data_ptr[i]);
  }

  return crc;
}

uint32_t Protocol::CRC32Calc(const uint8_t *data_ptr, size_t length) {
  size_t i;
  uint32_t crc = CRC_INIT;

  for (i = 0; i < length; i++) {
    crc = CRC32Update(crc, data_ptr[i]);
  }

  return crc;
}

bool Protocol::CRCHeadCheck(uint8_t *data_ptr, size_t length) {
  if (CRC16Calc(data_ptr, length) == 0) {
    return true;
  } else {
    return false;
  }
}

bool Protocol::CRCTailCheck(uint8_t *data_ptr, size_t length) {
  if (CRC32Calc(data_ptr, length) == 0) {
    return true;
  } else {
    return false;
  }
}
}
