#include <stdexcept>
#include <iostream>
#include "DTMFDecoder.h"

struct WAVHeader
{
  char chunkId[4] = {'R','I','F','F'};
  unsigned long chunkSize;
  char format[4] = {'W','A','V','E'};
  char subchunk1Id[4] = {'f','m','t',' '};
  unsigned long subchunk1Size = 16;
  uint16_t audioFormat = 1;
  uint16_t numChannels;
  unsigned long sampleRate;
  unsigned long byteRate;
  uint16_t blockAlign;
  uint16_t bitsPerSample;
  char subchunk2Id[4] = {'d','a','t','a'};
  unsigned long subchunk2Size = 0;
};


FILE * loadWaveFile(const std::string& filename, WAVHeader& header)
{
  FILE * fd = fopen(filename.c_str(), "rb");
  if (fd == nullptr) {
    std::cout << "Wave file not found!" << std::endl;
    exit(1);
  }
  fread(&header, sizeof(WAVHeader), 1, fd);
  return fd;
}

class TextHandler : public DTMFDecoder<>::Handler
{
  void OnCodeBegin(DTMFDecoder<> * const sender, uint8_t code)
  {
    std::cout << "Code Begin: " <<  code << std::endl;
  }

  void OnCode(DTMFDecoder<> * const sender, uint8_t code, size_t duration)
  {
    std::cout << "Code Beeping: " << code << " " << duration << std::endl;
  }

  void OnCodeEnd(DTMFDecoder<> * const sender, uint8_t code, size_t duration)
  {
    std::cout << "Code End: " << code << " " << duration << std::endl;
  }
};


int main (int argc, char *argv[])
{
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " something.wav" << std::endl;
    return 1;
  }

  WAVHeader header;

  FILE * fp = loadWaveFile(argv[1], header);

  size_t timestamp = 0;
  size_t read_length = 0;

  std::vector<int16_t> frames(400);

  TextHandler handler;
  DTMFDecoder<> decoder(&handler);
  std::cout << "Detecting begin..." << std::endl;
  while (read_length < header.subchunk2Size) {
    int chunk_length = fread(frames.data(), sizeof(int16_t), frames.size(), fp);

    if (chunk_length <= 0) {
      break;
    }
    read_length += chunk_length * sizeof(int16_t);
    decoder.decode(frames, chunk_length);
  }

  fclose(fp);
  std::cout << "Detecting complete!" << std::endl;
  return 0;
}