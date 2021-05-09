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
  auto h=std::make_shared<roborts_sdk::Handle>("/dev/serial_sdk");
  
  if(!h->Init()) return 1;
  int count = 3;

  /*-----------Subscriber Test-------------*/
  auto func = [&count] (const std::shared_ptr<roborts_sdk::cmd_chassis_info> message) -> void{
    std::cout<<"chassis_msg_"<<count<<" : " << std::hex <<(int)(message.get()->position_x_mm) << std::endl;
    count++;
  };

  // auto func2 = [&count] (const std::shared_ptr<roborts_sdk::cmd_uwb_info> message) -> void{
  //   std::cout<<"chassis_msg_"<<count<<" : "<<(int)message->error;
  //   count++;
  // };
  auto func3 = [&count] (const std::shared_ptr<roborts_sdk::cmd_gimbal_info> message) -> void{
    std::cout<<"chassis_msg_"<<count<<" : "<<(float)message->pitch_ecd_angle;
    count++;
  };

  auto sub1=h->CreateSubscriber<roborts_sdk::cmd_chassis_info>(CHASSIS_CMD_SET,CMD_PUSH_CHASSIS_INFO,CHASSIS_ADDRESS,MANIFOLD2_ADDRESS,func);
  // auto sub2=h->CreateSubscriber<roborts_sdk::cmd_uwb_info>(CHASSIS_CMD_SET,CMD_PUSH_UWB_INFO,CHASSIS_ADDRESS,MANIFOLD2_ADDRESS,func2);
  auto sub3=h->CreateSubscriber<roborts_sdk::cmd_gimbal_info>(GIMBAL_CMD_SET,CMD_PUSH_GIMBAL_INFO,GIMBAL_ADDRESS,BROADCAST_ADDRESS,func3);

  /*-----------Publisher Test-------------*/
  // auto pub1 = h->CreatePublisher<roborts_sdk::cmd_chassis_speed>(CHASSIS_CMD_SET,CMD_SET_CHASSIS_SPEED,MANIFOLD2_ADDRESS,CHASSIS_ADDRESS);
  // roborts_sdk::cmd_chassis_speed chassis_speed;
  // chassis_speed.rotate_x_offset=0;
  // chassis_speed.rotate_y_offset=0;
  // chassis_speed.vx=0x1234;
  // chassis_speed.vy=0;
  // chassis_speed.vw=0;
  // pub1->Publish(chassis_speed);


   /*-----------Client Test-------------*/
  // auto client1=h->CreateClient<chassis_mode_e,chassis_mode_e>(CHASSIS_CMD_SET,SET_CHASSIS_MODE,MANIFOLD2_ADDRESS,CHASSIS_ADDRESS);
  // auto mode = std::make_shared<chassis_mode_e>(SEPARATE_GIMBAL);

  // client1->AsyncSendRequest(mode,[](Client<chassis_mode_e,chassis_mode_e>::SharedFuture){
  //   std::cout<<"get!"<<std::endl;
  // });

  while(true){
    h->Spin();
    
    sleep(1);
    
  }


}