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

#include "execution.h"

namespace roborts_sdk {
Executor::Executor(std::shared_ptr<Handle> handle) : handle_(handle) {}
std::shared_ptr<Handle> Executor::GetHandle() {
  return handle_;
}

void Executor::ExecuteSubscription(const std::shared_ptr<SubscriptionBase>& subscription) {
  auto message_header = subscription->CreateMessageHeader();
  std::shared_ptr<void> message = subscription->CreateMessage();  // void类型指针，SubscriptionBase类型的指针可以调用其派生类的成员函数

  bool ret = GetHandle()->GetProtocol()->Take(  //handle作为分发层句柄，操作协议里的take函数，从接受池中取出一个消息
      subscription->GetCommandInfo().get(), //期望接收到的消息类型，CommandInfo结构体类型，包含命令码、是否需要ack等信息，GetCommandInfo返回的是对象的共享指针，get方法返回的是对象
      message_header.get(), //实际接收到的消息头
      message.get());       //实际接收到的消息数据
  if (ret) {
    subscription->HandleMessage(message_header, message);   //处理消息，调用回调函数
  } else {
//      DLOG_ERROR<<"take message failed!";
  }
  //TODO: add return message;
  //subscription->return_message(message);
}
void Executor::ExecuteService(const std::shared_ptr<ServiceBase>& service) {
  auto request_header = service->CreateRequestHeader();
  std::shared_ptr<void> request = service->CreateRequest();

  bool ret = GetHandle()->GetProtocol()->Take(
      service->GetCommandInfo().get(),
      request_header.get(),
      request.get());
  if (ret) {
    service->HandleRequest(request_header, request);
  } else {
    DLOG_ERROR << "take request failed!";
  }
}
void Executor::ExecuteClient(const std::shared_ptr<ClientBase>& client) {
  auto request_header = client->CreateRequestHeader();
  std::shared_ptr<void> response = client->CreateResponse();

  bool ret = GetHandle()->GetProtocol()->Take(
      client->GetCommandInfo().get(),
      request_header.get(),
      response.get());

  if (ret) {
    client->HandleResponse(request_header, response);
  } else {
//      DLOG_ERROR<<"take response failed!";
  }
}
}