#pragma once

#include <vector>
#include <map>
#include <math.h>

template<typename T = int16_t>
class DTMFDecoder
{
    public:
      typedef uint16_t Frequency;
      const Frequency kUndefinedFrequency = 0;


      class Handler {
        public:
          virtual void OnCodeBegin(DTMFDecoder * const sender, uint8_t code) {};
          virtual void OnCode(DTMFDecoder * const sender, uint8_t code, size_t duration) = 0;
          virtual void OnCodeEnd(DTMFDecoder * const sender, uint8_t code, size_t duration) {};
      };
    private:
      const std::vector<Frequency> kLowFreqs_ =  {697, 770, 852, 941};
      const std::vector<Frequency> kHighFreqs_ =  {1209, 1336,1477,1633};
      const uint8_t kDtmfCodes_[4][4] = {
                                          {'1','2','3','A'},
                                          {'4','5','6','B'},
                                          {'7','8','9','C'},
                                          {'*','0','#','D'}
                                        };
      Handler * handler_ = nullptr;
      size_t sampleRate_;
      size_t samplesPerMilliseconds_;
      size_t windowDurationMilliseconds_;
      size_t framesCount_;
      size_t overlapFramesCount_;
      size_t timestamp_;
      double thresold_;
      std::vector<T> windowFrames_;
      std::vector<double> windowFunctionTable_;
      typedef std::pair<Frequency, Frequency> FrequencyPair;
      struct FrequencyPairStat {
          size_t duration;
          uint8_t code;
      };
      std::map<FrequencyPair, FrequencyPairStat> dtmfStatsTable_;

      double goertzelFunction(size_t frequency, const std::vector<double>& samples)
      {
          size_t scalingFactor = samples.size() / 2;
          double k = 0.5 + (((double)samples.size() * frequency) / sampleRate_),
                 omega = (2.0 * M_PI * k) / samples.size(),
                 sine = sin(omega),
                 cosine =cos(omega),
                 coeff = 2.0 * cosine,
                 q0 = 0,
                 q1 = 0,
                 q2 = 0;

          for (auto sample : samples) {
              q0 = coeff * q1 - q2 + sample;
              q2 = q1;
              q1 = q0;
          }
          double real = (q1 - q2 * cosine) / scalingFactor;
          double imag = (q2 * sine) / scalingFactor;
          return sqrtf(real*real + imag*imag);
      }

      void createWindowFunctionTable()
      {
          windowFunctionTable_.resize(framesCount_);
          for (size_t i = 0; i < framesCount_; ++i) {
            windowFunctionTable_[i] = (0.42 - 0.5 * cos(2.0 * M_PI * i / (framesCount_ - 1)) + 0.08 * cos(4.0 * M_PI * i / (framesCount_ - 1)))
                                      /
                                      (pow(2, sizeof(T) * 8 - 1) - 1);
          }
      }

      void fillDtmfStatsTable()
      {
        dtmfStatsTable_.clear();
        size_t idxl = 0;
        for (auto l : kLowFreqs_) {
          size_t idxh = 0;
          for (auto h : kHighFreqs_) {
            uint8_t c = kDtmfCodes_[idxl][idxh];
            dtmfStatsTable_.insert(std::make_pair(std::make_pair(l,h), FrequencyPairStat({0, c})));
            ++idxh;
          }
          ++idxl;
        }
      }

      Frequency getExistsFrequency(const std::vector<double>& frames, const std::vector<Frequency>& frequencies)
      {
        double max = thresold_;
        Frequency ret = kUndefinedFrequency;
        for (auto f : frequencies) {
          double mag = 20.0 * log10(goertzelFunction(f , frames));
          if (mag > max) {
            max = mag;
            ret = f;
          }
        }
        return ret;
      }

      void truncateDtmfStatTable(const FrequencyPair* ignore_pair = nullptr)
      {
        for (auto& kv : dtmfStatsTable_) {
          if (ignore_pair && kv.first == *ignore_pair) continue;
          if (kv.second.duration >= framesCount_) {
            handler_->OnCodeEnd(this,kv.second.code, kv.second.duration);
          }
          kv.second.duration = 0;
        }
      }

    public:
      DTMFDecoder(Handler* handler, size_t sampleRate = 8000, size_t windowDurationMilliseconds = 50, size_t windowOverlapMilliseconds = 10, double thresold = -59.0)
        : handler_(handler)
        , sampleRate_(sampleRate)
        , windowDurationMilliseconds_(windowDurationMilliseconds)
        , thresold_(thresold)
      {
          if (windowOverlapMilliseconds > windowDurationMilliseconds_) {
            windowOverlapMilliseconds = windowDurationMilliseconds_;
          }
          samplesPerMilliseconds_ = sampleRate_ / 1000;
          framesCount_ = samplesPerMilliseconds_* windowDurationMilliseconds_;
          overlapFramesCount_ = samplesPerMilliseconds_ * windowOverlapMilliseconds;
          windowFrames_.reserve(framesCount_);

          createWindowFunctionTable();
          reset();
      };

      void reset()
      {
        timestamp_ = 0;
        windowFrames_.clear();
        fillDtmfStatsTable();
      }

      void decode(const std::vector<T>& frames, size_t framesCount)
      {
        size_t offset = 0;
        while (framesCount > 0) {
          size_t len = std::min(framesCount, framesCount_ - windowFrames_.size());
          windowFrames_.insert(windowFrames_.end(), frames.begin() + offset, frames.begin() + offset + len);
          offset += len;
          framesCount -= len;
          if (windowFrames_.size() >= framesCount_) {
              std::vector<double> resultedFrames(framesCount_);
              for (size_t i = 0; i < framesCount_; ++i) {
                resultedFrames[i] = windowFunctionTable_[i] * windowFrames_[i];
              }
              Frequency low = getExistsFrequency(resultedFrames, kLowFreqs_),
                        high = getExistsFrequency(resultedFrames, kHighFreqs_);

              if (low == kUndefinedFrequency || high == kUndefinedFrequency) {
                truncateDtmfStatTable();
              } else {
                  auto r = dtmfStatsTable_.find(std::make_pair(low, high));
                  truncateDtmfStatTable(&r->first);
                  auto duration = r->second.duration;
                  r->second.duration += overlapFramesCount_;
                  if (duration < framesCount_ && r->second.duration >= framesCount_) {
                    handler_->OnCodeBegin(this, r->second.code);
                  } else if (r->second.duration % framesCount_ == 0) {
                    handler_->OnCode(this, r->second.code, r->second.duration / samplesPerMilliseconds_);
                  }
              }

              windowFrames_.erase(windowFrames_.begin(), windowFrames_.begin() + overlapFramesCount_);
          }
          timestamp_ += len;
        }
      }
};