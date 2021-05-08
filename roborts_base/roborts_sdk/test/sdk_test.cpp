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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <iostream>

#include "../sdk.h"
#include "../protocol/protocol_define.h"

int main() {

  // roborts_sdk::SerialDevice s("/dev/serial_sdk", 921600);

  // uint8_t bb[2]={65,66};
  // uint8_t aa[2]={0};

  // s.Init();

  // s.Write(bb, 2);

  // while (true)
  // {

  //   s.Write(bb, 2);
  //   s.Read(aa,2);
  //   std::cout << aa[0] << aa[1] << std::endl;
  // }




  auto h = std::make_shared<roborts_sdk::Handle>("/dev/serial_sdk");
  
  if(!h->Init()) return 1;

  int count = 1;
  
  /*-----------Subscriber Test-------------*/
  std::cout << "---hello-Subscriber---" << count << std::endl;
  auto func = [&count] (const std::shared_ptr<roborts_sdk::cmd_chassis_speed> message) -> void {
    std::cout <<"chassis_msg_"<<count<<" : "<<(int)(message->vx);
    count++;
  };
  // auto func2 = [&count] (const std::shared_ptr<roborts_sdk::p_uwb_data_t> message) -> void{
  //   DLOG_INFO<<"chassis_msg_"<<count<<" : "<<(int)message->error;
  //   count++;
  // };
  // auto func3 = [&count] (const std::shared_ptr<roborts_sdk::cmd_gimbal_info> message) -> void{
  //   std::cout <<"chassis_msg_"<<count<<" : "<<(int16_t)message->pitch_rate;
  //   count++;
  // };
  auto sub1=h->CreateSubscriber<roborts_sdk::cmd_chassis_speed>(CHASSIS_CMD_SET,CMD_SET_CHASSIS_SPEED,MANIFOLD2_ADDRESS,CHASSIS_ADDRESS,func);
  // auto sub1=h->CreateSubscriber<roborts_sdk::cmd_chassis_speed>(CHASSIS_CMD_SET,CMD_PUSH_CHASSIS_INFO,CHASSIS_ADDRESS,MANIFOLD2_ADDRESS,func);
  // auto sub2=h->CreateSubscriber<p_uwb_data_t>(CHASSIS_CMD_SET,CMD_PUSH_UWB_INFO,CHASSIS_ADDRESS,MANIFOLD2_ADDRESS,func2);
  // auto sub3=h->CreateSubscriber<roborts_sdk::cmd_gimbal_info>(GIMBAL_CMD_SET,CMD_PUSH_GIMBAL_INFO,GIMBAL_ADDRESS,BROADCAST_ADDRESS,func3);


  // /*-----------Publisher Test-------------*/
  std::cout << "---hello-Publisher---" << count << std::endl;
  auto pub1 = h->CreatePublisher<roborts_sdk::cmd_chassis_speed>(CHASSIS_CMD_SET,CMD_SET_CHASSIS_SPEED,MANIFOLD2_ADDRESS,CHASSIS_ADDRESS);
  roborts_sdk::cmd_chassis_speed chassis_speed;
  chassis_speed.vx=0x0102;
  chassis_speed.vy=0x0304;
  chassis_speed.vw=0x0506;
  chassis_speed.rotate_x_offset=0x0708;
  chassis_speed.rotate_y_offset=0x090a;
  pub1->Publish(chassis_speed);
  

  //  /*-----------Client Test-------------*/
  // auto client1=h->CreateClient<chassis_mode_e,chassis_mode_e>(CHASSIS_CMD_SET,SET_CHASSIS_MODE,MANIFOLD2_ADDRESS,CHASSIS_ADDRESS);
  // auto mode = std::make_shared<chassis_mode_e>(SEPARATE_GIMBAL);

  // client1->AsyncSendRequest(mode,[](Client<chassis_mode_e,chassis_mode_e>::SharedFuture){
  //   std::cout<<"get!"<<std::endl;
  // });

  while(true){
    h->Spin();
    sleep(1);
    
  pub1->Publish(chassis_speed);
  }


}