#include <sys/time.h>

#include "opencv2/opencv.hpp"

#include "cyberdog_vision/semaphore_op.hpp"
#include "cyberdog_vision/shared_memory_op.hpp"

#define SHM_PROJ_ID 'A'
#define SEM_PROJ_ID 'B'

class ImgProduce
{

public:
  ImgProduce();
  ~ImgProduce();

  int Produce(const cv::Mat & img);

private:
  int Init();

  int shm_id_;
  int sem_set_id_;
  char * shm_addr_;
};

ImgProduce::ImgProduce()
: shm_addr_(nullptr)
{
  if (0 != Init()) {
    throw std::logic_error("Init fail. ");
  }
}

ImgProduce::~ImgProduce()
{
  cyberdog_vision::DetachShm(shm_addr_);
}

int ImgProduce::Init()
{
  if (0 != cyberdog_vision::CreateShm(SHM_PROJ_ID, sizeof(uint64_t) + IMAGE_SIZE, shm_id_)) {
    std::cout << "Failed to create shared memory." << std::endl;
    return -1;
  }
  shm_addr_ = cyberdog_vision::GetShmAddr(shm_id_, sizeof(uint64_t) + IMAGE_SIZE);
  if (shm_addr_ == nullptr) {
    std::cout << "Failed to map shared memory." << std::endl;
    return -1;
  }

  if (0 != cyberdog_vision::CreateSem(SEM_PROJ_ID, 3, sem_set_id_)) {
    std::cout << "Failed to create shared memory semaphore." << std::endl;
    return -1;
  }

  return 0;
}

int ImgProduce::Produce(const cv::Mat & img)
{
  if (shm_addr_ == nullptr) {
    return -1;
  }

  cyberdog_vision::WaitSem(sem_set_id_, 1);
  cyberdog_vision::WaitSem(sem_set_id_, 0);
  struct timeval tv;
  gettimeofday(&tv, 0);
  uint64_t time = tv.tv_sec * 1000 * 1000 * 1000 + tv.tv_usec / 1000;
  memcpy(shm_addr_, &time, sizeof(uint64_t));
  memcpy(shm_addr_ + sizeof(uint64_t), img.data, IMAGE_SIZE);
  cyberdog_vision::SignalSem(sem_set_id_, 0);
  cyberdog_vision::SignalSem(sem_set_id_, 2);

  return 0;
}

int main(int argc, char ** argv)
{
  ImgProduce * proc_ = new ImgProduce();
  if (argc < 2) {
    std::cout << "Please input the video path. " << std::endl;
    return -1;
  }
  cv::VideoCapture cap(argv[1]);
  if (!cap.isOpened()) {
    std::cout << "Open video file fail." << std::endl;
    return -1;
  }

  int i = 0;
  while (true) {
    std::cout << "Frame: " << i++ << std::endl;
    cv::Mat img;
    cap >> img;
    cv::resize(img, img, cv::Size(640, 480));
    if (0 != proc_->Produce(img)) {
      return -1;
    }
    cv::waitKey(50);
  }

  delete proc_;
  return 0;
}
