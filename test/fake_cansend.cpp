#include <chrono>
#include <thread>
#include <string>
int main(int argc,char** argv){
  if(argc!=3||std::string(argv[1])!="fake0")return 8;
  const std::string f=argv[2];
  if(f=="200#9CFF64000CFE0001")return 0;
  if(f=="200#0000000000000100")return 7;
  if(f=="200#0000000000000200"){std::this_thread::sleep_for(std::chrono::seconds(2));return 0;}
  return 9;
}
