ps -ef |grep ros |grep -v "grep" |awk '{print $2}'  |xargs kill -9
