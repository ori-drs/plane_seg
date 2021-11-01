#include <thread>
#include <condition_variable>
#include <mutex>
#include <sstream>

//#include <lcm/lcm-cpp.hpp>
//#include <ConciseArgs>

//#include <drc_utils/LcmWrapper.hpp>
//#include <drc_utils/BotWrapper.hpp>

//#include <bot_lcmgl_client/lcmgl.h>

//#include <lcmtypes/drc/map_scans_t.hpp>
//#include <lcmtypes/drc/affordance_collection_t.hpp>
//#include <lcmtypes/drc/block_fit_request_t.hpp>

//#include <maps/ScanBundleView.hpp>
//#include <maps/LcmTranslator.hpp>

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>

#include "plane_seg/BlockFitter.hpp"

struct State {
  //drc::BotWrapper::Ptr mBotWrapper;
  //drc::LcmWrapper::Ptr mLcmWrapper;
  bool mRunContinuously;
  bool mDoFilter;
  bool mRemoveGround;
  bool mGrabExactPoses;
  bool mDebug;
  Eigen::Vector3f mBlockSize;
  int mAlgorithm;
  bool mDoTrigger;
  std::string mNamePrefix;
  bool mTriggered;
  bool mShouldProcess;

  //drc::map_scans_t mData;
  int64_t mLastDataTime;
  Eigen::Isometry3f mSensorPose;
  Eigen::Isometry3f mGroundPose;

  std::thread mWorkerThread;
  std::condition_variable mCondition;
  std::mutex mProcessMutex;
  std::mutex mDataMutex;

  State() {
    mRunContinuously = false;
    mDoFilter = true;
    mRemoveGround = true;
    mGrabExactPoses = false;
    mDebug = false;
    mAlgorithm = planeseg::BlockFitter::RectangleFitAlgorithm::MinimumArea;
    mBlockSize << 15+3/8.0, 15+5/8.0, 5+5/8.0;
    mBlockSize *=0.0254;
    mDoTrigger = false;
    mNamePrefix = "cinderblock";
    mTriggered = true;
    mShouldProcess = false;
  }

  void start() {
    mLastDataTime = 0;
    //mData.utime = 0;
    //mLcmWrapper->get()->subscribe("MAP_SCANS", &State::onScans, this);
    mWorkerThread = std::thread(std::ref(*this));
    //mLcmWrapper->startHandleThread(true);
  }

  void stop() {
    //mLcmWrapper->stopHandleThread();
    if (mWorkerThread.joinable()) mWorkerThread.join();
  }

  void operator()() {
    while (true) {
      // wait for data
      std::unique_lock<std::mutex> lock(mProcessMutex);
      mCondition.wait_for(lock, std::chrono::milliseconds(100));
      if (!mShouldProcess) continue;
      mShouldProcess = false;

      // grab data
      //drc::map_scans_t data;
      Eigen::Isometry3f sensorPose;
      Eigen::Isometry3f groundPose;
      /*
      {
        std::unique_lock<std::mutex> dataLock(mDataMutex);
        if (mData.utime == mLastDataTime) continue;
        data = mData;
        sensorPose = mSensorPose;
        groundPose = mGroundPose;
        mLastDataTime = mData.utime;
      }
      */

      // convert scans to point cloud
      //maps::ScanBundleView view;
      //maps::LcmTranslator::fromLcm(data, view);
      //maps::PointCloud rawCloud, curCloud;
      std::vector<float> allDeltas;
      /*
      for (const auto& scan : view.getScans()) {

        // compute range deltas
        int numRanges = scan->getNumRanges();
        std::vector<float> deltas;
        const auto& ranges = scan->getRanges();
        float prevRange = -1;
        int curIndex = 0;
        for (int i = 0; i < numRanges; ++i, ++curIndex) {
          if (ranges[i] <= 0) continue;
          prevRange = ranges[i];
          deltas.push_back(0);
          break;
        }
        for (int i = curIndex+1; i < numRanges; ++i) {
          float range = ranges[i];
          if (range <= 0) continue;
          deltas.push_back(range-prevRange);
          prevRange = range;
        }

        // add this scan to cloud
        scan->get(curCloud, true);
        rawCloud += curCloud;
        allDeltas.insert(allDeltas.end(), deltas.begin(), deltas.end());
      }
      pcl::transformPointCloud
        (rawCloud, rawCloud,
         Eigen::Affine3f(view.getTransform().matrix()).inverse());
      planeseg::LabeledCloud::Ptr cloud(new planeseg::LabeledCloud());
      */
      //pcl::copyPointCloud(rawCloud, *cloud);
      /* TODO: change point type
      for (int i = 0; i < (int)cloud->size(); ++i) {
        cloud->points[i].label = 1000*allDeltas[i];
      }
      */

      /*
      // remove points outside max radius
      const float kValidRadius = 5;  // meters; TODO: could make this a param
      const float kValidRadius2 = kValidRadius*kValidRadius;
      planeseg::LabeledCloud::Ptr tempCloud(new planeseg::LabeledCloud());
      for (int i = 0; i < (int)cloud->size(); ++i) {
        Eigen::Vector3f p = cloud->points[i].getVector3fMap();
        float dist2 = (p-sensorPose.translation()).squaredNorm();
        if (dist2 > kValidRadius2) continue;
        tempCloud->push_back(cloud->points[i]);
      }
      std::swap(cloud, tempCloud);

      // process
      planeseg::BlockFitter fitter;
      fitter.setSensorPose(sensorPose.translation(),
                           sensorPose.rotation().col(2));
      fitter.setGroundBand(groundPose.translation()[2]-1.0,
                           groundPose.translation()[2]+0.5);
      if (mDoFilter) fitter.setAreaThresholds(0.8, 1.2);
      else fitter.setAreaThresholds(0, 1000);
      fitter.setBlockDimensions(mBlockSize);
      fitter.setRemoveGround(mRemoveGround);
      fitter.setRectangleFitAlgorithm
        ((planeseg::BlockFitter::RectangleFitAlgorithm)mAlgorithm);
      fitter.setDebug(mDebug);
      fitter.setCloud(cloud);
      auto result = fitter.go();
      if (!result.mSuccess) {
        std::cout << "error: could not detect blocks" << std::endl;
        continue;
      }

      //
      // construct json string
      //

      // convenience methods
      auto vecToStr = [](const Eigen::Vector3f& iVec) {
        std::ostringstream oss;
        oss << iVec[0] << ", " << iVec[1] << ", " << iVec[2];
        return oss.str();
      };
      auto rotToStr = [](const Eigen::Matrix3f& iRot) {
        std::ostringstream oss;
        Eigen::Quaternionf q(iRot);
        oss << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z();
        return oss.str();
      };

      // header
      std::string json;
      json += "{\n";
      json += "  \"command\": \"echo_response\",\n";
      json += "  \"descriptions\": {\n";
      std::string timeString = std::to_string(mBotWrapper->getCurrentTime());

      // blocks
      for (int i = 0; i < (int)result.mBlocks.size(); ++i) {
        const auto& block = result.mBlocks[i];
        std::string dimensionString = vecToStr(block.mSize);
        std::string positionString = vecToStr(block.mPose.translation());
        std::string quaternionString = rotToStr(block.mPose.rotation());
        Eigen::Vector3f color(0.5, 0.4, 0.5);
        std::string colorString = vecToStr(color);
        float alpha = 1.0;
        std::string uuid = timeString + "_" + std::to_string(i+1);
        
        json += "    \"" + uuid + "\": {\n";
        json += "      \"classname\": \"BoxAffordanceItem\",\n";
        json += "      \"pose\": [[" + positionString + "], [" +
          quaternionString + "]],\n";
        json += "      \"uuid\": \"" + uuid + "\",\n";
        json += "      \"Dimensions\": [" + dimensionString + "],\n";
        json += "      \"Color\": [" + colorString + "],\n";
        json += "      \"Alpha\": " + std::to_string(alpha) + ",\n";
        json += "      \"Name\": \"" + mNamePrefix + " " +
          std::to_string(i) + "\"\n";
        json += "    },\n";
      }
      */



      /*
      // ground
      {
        std::string positionString, quaternionString, dimensionString;
        Eigen::Vector3f groundNormal = result.mGroundPlane.head<3>();
        Eigen::Vector3f groundSize(100,100,0.01);
        {
          Eigen::Vector3f p = groundPose.translation();
          p -= (groundNormal.dot(p)+result.mGroundPlane[3])*groundNormal;
          p -= (groundSize[2]/2)*groundNormal;
          positionString = vecToStr(p);
        }
        {
          std::ostringstream oss;
          Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
          rot.col(2) = groundNormal.normalized();
          rot.col(1) = rot.col(2).cross(Eigen::Vector3f::UnitX()).normalized();
          rot.col(0) = rot.col(1).cross(rot.col(2)).normalized();
          quaternionString = rotToStr(rot);
        }
        dimensionString = vecToStr(groundSize);
        
        json += "    \"ground affordance\": {\n";
        json += "      \"classname\": \"BoxAffordanceItem\",\n";
        json += "      \"pose\": [[" + positionString + "], [" +
          quaternionString + "]],\n";
        json += "      \"uuid\": \"ground affordance\",\n";
        json += "      \"Dimensions\": [" + dimensionString+ "],\n";
        json += "      \"Name\": \"ground affordance\",\n";
        json += "      \"Visible\": 0\n";
        json += "    }\n";
      }

      // footer
      json += "  },\n";
      json += "  \"commandId\": \"" + timeString + "\",\n";
      json += "  \"collectionId\": \"block-fitter\"\n";
      json += "}\n";
      */

      // publish result
      /*
      drc::affordance_collection_t msg;
      msg.utime = data.utime;
      msg.name = json;
      msg.naffs = 0;
      mLcmWrapper->get()->publish("AFFORDANCE_COLLECTION_COMMAND", &msg);
      */
      std::cout << "Published affordance collection" << std::endl;

      /*
      // publish lcmgl
      if (mDebug) {
        bot_lcmgl_t* lcmgl;
        lcmgl = bot_lcmgl_init(mLcmWrapper->get()->getUnderlyingLCM(),
                              "block-fitter");
        for (const auto& block : result.mBlocks) {
          bot_lcmgl_color3f(lcmgl, 1, 0, 0);
          bot_lcmgl_line_width(lcmgl, 4);
          bot_lcmgl_begin(lcmgl, LCMGL_LINE_LOOP);
          for (const auto& pt : block.mHull) {
            bot_lcmgl_vertex3f(lcmgl, pt[0], pt[1], pt[2]);
          }
          bot_lcmgl_end(lcmgl);
        }

        bot_lcmgl_color3f(lcmgl, 0, 1, 0);
        bot_lcmgl_begin(lcmgl, LCMGL_LINE_LOOP);
        for (const auto& pt : result.mGroundPolygon) {
          bot_lcmgl_vertex3f(lcmgl, pt[0], pt[1], pt[2]);
        }
        bot_lcmgl_end(lcmgl);

        bot_lcmgl_switch_buffer(lcmgl);
        bot_lcmgl_destroy(lcmgl);
      }
      */

      if (!mRunContinuously) break;
    }
    //mLcmWrapper->stopHandleThread();
  }


  /*
  void onScans(const lcm::ReceiveBuffer* iBuf,
               const std::string& iChannel,
               const drc::map_scans_t* iMessage) {
    std::unique_lock<std::mutex> lock(mDataMutex);
    if (!mTriggered) return;
    mData = *iMessage;
    int64_t scanTime = iMessage->utime;
    int64_t headPoseTime = mBotWrapper->getLatestTime("head", "local");
    int64_t groundPoseTime = mBotWrapper->getLatestTime("ground", "local");
    if ((groundPoseTime == 0) || (headPoseTime == 0) ||
        (mGrabExactPoses && ((std::abs(headPoseTime-scanTime) > 1e6) ||
                             (std::abs(groundPoseTime-scanTime) > 1e6)))) {
      std::cout << "warning: got scans but no valid pose found" << std::endl;
      return;
    }
    mBotWrapper->getTransform("head", "local", mSensorPose, iMessage->utime);
    mBotWrapper->getTransform("ground", "local", mGroundPose, iMessage->utime);
    mShouldProcess = true;
    mCondition.notify_one();
    if (mDoTrigger) mTriggered = false;
  }
  */

  /*
  void onTrigger(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel,
                 const drc::block_fit_request_t* iMessage) {
    mNamePrefix = iMessage->name_prefix;
    mBlockSize << iMessage->dimensions[0], iMessage->dimensions[1],
      iMessage->dimensions[2];
    mAlgorithm = iMessage->algorithm;
    mLastDataTime = 0;  // force processing of new data
    mTriggered = true;
    std::cout << "received trigger" << std::endl;
  }
  */
};

int main(const int /*iArgc*/, const char** /*iArgv*/) {

  std::string sizeString("");
  std::string triggerChannel;
  State state;

  /*
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(state.mRunContinuously, "c", "continuous", "run continuously");
  opt.add(state.mDoFilter, "f", "filter", "filter blocks based on size");
  opt.add(sizeString, "s", "blocksize", "prior size for blocks \"x y z\"");
  opt.add(state.mRemoveGround, "g", "remove-ground",
          "whether to remove ground before processing");
  opt.add(state.mAlgorithm, "a", "algorithm",
          "0=min_area, 1=closest_size, 2=closest_hull");
  opt.add(state.mGrabExactPoses, "p", "exact-poses",
          "wait for synchronized poses");
  opt.add(triggerChannel, "t", "trigger-channel",
          "perform block fit only when trigger is received");
  opt.add(state.mDebug, "d", "debug", "debug flag");
  opt.parse();
  */

  if (sizeString.length() > 0) {
    std::istringstream iss(sizeString);
    float x, y, z;
    if (iss >> x) {
      if (iss >> y) {
        if (iss >> z) {
          state.mBlockSize << x,y,z;
          std::cout << "using block size " << state.mBlockSize.transpose() <<
            std::endl;
        }
      }
    }
  }

  //do {
  //  state.mBotWrapper.reset(new drc::BotWrapper());
  //}
  //while (state.mBotWrapper->getBotParam() == NULL);
  std::cout << "got bot params" << std::endl;
  //state.mLcmWrapper.reset(new drc::LcmWrapper(state.mBotWrapper->getLcm()));

  if (triggerChannel.length() > 0) {
    state.mDoTrigger = true;
    state.mTriggered = false;
    //state.mLcmWrapper->get()->subscribe(triggerChannel,
    //                                    &State::onTrigger, &state);
  }

  state.start();
  state.stop();

  return 1;
}
