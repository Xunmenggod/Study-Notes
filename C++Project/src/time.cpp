#include <chrono>
#include <iostream>
#include <thread>

using namespace std;

int main() {
  auto beginTime = std::chrono::high_resolution_clock::now();

  std::this_thread::sleep_for(std::chrono::seconds(1));

  auto endTime = std::chrono::high_resolution_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(
      endTime - beginTime);

  double cost = (double)elapsedTime.count();

  std::cout << "cost time : " << cost * 1e-6 << "s" << std::endl;

  // system("pause");
}