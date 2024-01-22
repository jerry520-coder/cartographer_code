-- Copyright 2017 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- 引入map_builder.lua文件，它定义了MAP_BUILDER对象，用来设置地图构建的选项
include "map_builder.lua"

-- 定义一个名为MAP_BUILDER_SERVER的表，用来存储服务器端的参数和选项
MAP_BUILDER_SERVER = {
  -- 设置map_builder的选项，它是一个MAP_BUILDER对象
  map_builder = MAP_BUILDER,
  -- 设置事件处理的线程数，值越大，处理能力越强，但消耗资源越多
  num_event_threads = 4,
  -- 设置grpc通信的线程数，值越大，通信能力越强，但消耗资源越多
  num_grpc_threads = 4,
  -- 设置服务器的地址和端口，一般为"0.0.0.0:50051"
  server_address = "0.0.0.0:50051",
  -- 设置上行服务器的地址，如果为空，表示不使用上行服务器
  uplink_server_address = "",
  -- 设置上传数据的批量大小，值越大，上传效率越高，但内存占用越多
  upload_batch_size = 100,
  -- 设置是否启用SSL加密，如果为true，需要提供证书和密钥文件
  enable_ssl_encryption = false,
  -- 设置是否启用Google认证，如果为true，需要提供客户端和服务器的认证文件
  enable_google_auth = false,
}

-- 设置是否按照轨迹进行整合，如果为true，会将同一轨迹的数据整合到一起，否则会按照时间顺序整合
MAP_BUILDER.collate_by_trajectory = true

