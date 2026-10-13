#include "stdio.h"
#include "SynthesizerTrn.h"
#include "utils.h"
#include "string.h"


void convertAudioToWavBuf(
    char * toBuf, 
    char * fromBuf,
    int totalAudioLen)
{
    char * header = toBuf;
    int byteRate = 16 * 16000 * 1 / 8;
    int totalDataLen = totalAudioLen + 36;
    int channels = 1;
    int  longSampleRate = 16000;

    header[0] = 'R'; // RIFF/WAVE header
    header[1] = 'I';
    header[2] = 'F';
    header[3] = 'F';
    header[4] = (char) (totalDataLen & 0xff);
    header[5] = (char) ((totalDataLen >> 8) & 0xff);
    header[6] = (char) ((totalDataLen >> 16) & 0xff);
    header[7] = (char) ((totalDataLen >> 24) & 0xff);
    header[8] = 'W';
    header[9] = 'A';
    header[10] = 'V';
    header[11] = 'E';
    header[12] = 'f'; // 'fmt ' chunk
    header[13] = 'm';
    header[14] = 't';
    header[15] = ' ';
    header[16] = 16; // 4 bytes: size of 'fmt ' chunk
    header[17] = 0;
    header[18] = 0;
    header[19] = 0;
    header[20] = 1; // format = 1
    header[21] = 0;
    header[22] = (char) channels;
    header[23] = 0;
    header[24] = (char) (longSampleRate & 0xff);
    header[25] = (char) ((longSampleRate >> 8) & 0xff);
    header[26] = (char) ((longSampleRate >> 16) & 0xff);
    header[27] = (char) ((longSampleRate >> 24) & 0xff);
    header[28] = (char) (byteRate & 0xff);
    header[29] = (char) ((byteRate >> 8) & 0xff);
    header[30] = (char) ((byteRate >> 16) & 0xff);
    header[31] = (char) ((byteRate >> 24) & 0xff);
    header[32] = (char) (1 * 16 / 8); // block align
    header[33] = 0;
    header[34] = 16; // bits per sample
    header[35] = 0;
    header[36] = 'd';
    header[37] = 'a';
    header[38] = 't';
    header[39] = 'a';
    header[40] = (char) (totalAudioLen & 0xff);
    header[41] = (char) ((totalAudioLen >> 8) & 0xff);
    header[42] = (char) ((totalAudioLen >> 16) & 0xff);
    header[43] = (char) ((totalAudioLen >> 24) & 0xff);

    memcpy(toBuf+44, fromBuf, totalAudioLen);

}

#include "string"
#include "Hanz2Piny.h"
#include "hanzi2phoneid.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cstdlib>

#include <string.h>
#include <ros/ros.h>
#include <std_msgs/String.h> 

using namespace std;

char* modelPath="/home/zq/summer_tts_ws/src/summer_tts/models/single_speaker_fast.bin";
//char* modelPath="/home/zq/summer_tts_ws/src/summer_tts/models/single_speaker_english_fast.bin";
//char* modelPath="/home/zq/summer_tts_ws/src/summer_tts/models/single_speaker_english.bin";
const char* audioPath="/home/zq/summer_tts_ws/src/summer_tts/audios/robot_audio.wav";
const char* playPath="play /home/zq/summer_tts_ws/src/summer_tts/audios/robot_audio.wav";
// 可在原 ROS 启动文件中设置这三个私有参数；默认仍对应上传代码的部署位置。
std::string configuredModelPath, configuredAudioPath, configuredPlayPath;

// ===== 2026-09-30 TTS播放完成同步 START =====
// Python 端需要知道机器人什么时候“真正播报结束”，否则会在播报未完成时开始录音。
// 这里新增 /summer_tts_done：playWav() 阻塞播放完成后发布原文本。
ros::Publisher tts_done_pub;
// ===== 2026-09-30 TTS播放完成同步 END =====

void playWav()
{
    if (system(playPath) != 0) {
        throw std::runtime_error("音频播放命令失败，不发布完成回执");
    }
}

void makeTextToWav(const std::string& text_content, const char* out_path)
{
    const Hanz2Piny hanz2piny;
    std::string line;

    std::istringstream text_stream(text_content);
    std::string sub_line;

    while(getline(text_stream, sub_line)) 
    {
        if (hanz2piny.isStartWithBom(sub_line)) 
        {
            sub_line = std::string(sub_line.cbegin() + 3, sub_line.cend());
        }
        line += sub_line + "  ";  
    }

    float * dataW = NULL;
    int32_t modelSize = ttsLoadModel(modelPath, &dataW);
    if (modelSize <= 0 || dataW == NULL) {
        if (dataW != NULL) tts_free_data(dataW);
        throw std::runtime_error("TTS合成模型加载失败");
    }

    SynthesizerTrn * synthesizer = new SynthesizerTrn(dataW, modelSize);

    int32_t spkNum = synthesizer->getSpeakerNum();
    
    printf("Available speakers in the model are %d\n",spkNum);

    if(spkNum > 20)
    {
        // 旧分支生成十个其它文件却播放旧 audioPath，会误发完成回执。
        delete synthesizer;
        tts_free_data(dataW);
        throw std::runtime_error("此部署应使用上传源码指定的单说话人TTS模型");
    }
    else
    {
        int32_t retLen = 0;
        int16_t * wavData = synthesizer->infer(line,0, 1.0,retLen);
        if (retLen <= 0 || wavData == NULL) {
            if (wavData != NULL) tts_free_data(wavData);
            delete synthesizer;
            tts_free_data(dataW);
            throw std::runtime_error("TTS合成没有有效音频");
        }

        char * dataForFile = new char[retLen*sizeof(int16_t)+44];
        convertAudioToWavBuf(dataForFile, (char *)wavData, retLen*sizeof(int16_t));

        FILE * fpOut = fopen(out_path,"wb");
        bool written = false;
        if (fpOut != NULL) {
            written = fwrite(dataForFile, retLen*sizeof(int16_t)+44, 1, fpOut) == 1;
            written = (fclose(fpOut) == 0) && written;
        }
        delete[] dataForFile;
        tts_free_data(wavData);
        if (!written) {
            delete synthesizer;
            tts_free_data(dataW);
            throw std::runtime_error("TTS音频文件写入失败");
        }
    }

    delete synthesizer;
    tts_free_data(dataW);
}

void summer_tts_topicCallBack(const std_msgs::String::ConstPtr& msg)
{
	std::cout << "get topic text:" << msg->data.c_str() << std::endl;

    try {
        makeTextToWav(msg->data.c_str(), audioPath);
        playWav();
    } catch (const std::exception& error) {
        ROS_ERROR("TTS失败：%s", error.what());
        return;
    }

	// ===== 2026-09-30 TTS播放完成同步 START =====
	// playWav() 返回后认为本次播报完成，通知 Python 可以开始录音。
	std_msgs::String done_msg;
	done_msg.data = msg->data;
	tts_done_pub.publish(done_msg);
	std::cout << "tts done:" << msg->data.c_str() << std::endl;
	// ===== 2026-09-30 TTS播放完成同步 END =====
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"summer_tts_sub_node");
	ros::NodeHandle nd;
    ros::NodeHandle privateNode("~");
    privateNode.param<std::string>("model_path", configuredModelPath, std::string(modelPath));
    privateNode.param<std::string>("audio_path", configuredAudioPath, std::string(audioPath));
    privateNode.param<std::string>("play_command", configuredPlayPath, std::string(playPath));
    modelPath = const_cast<char*>(configuredModelPath.c_str());
    audioPath = configuredAudioPath.c_str();
    playPath = configuredPlayPath.c_str();
    // 主流程发送初始化播报；不在节点建立连接前额外播放无关英文。
	// ===== 2026-09-30 TTS播放完成同步 START =====
	// 非 latch，避免新订阅者收到旧的 done 消息。
	tts_done_pub = nd.advertise<std_msgs::String>("summer_tts_done", 3);
	// ===== 2026-09-30 TTS播放完成同步 END =====
	ros::Subscriber sub = nd.subscribe("summer_tts_topic", 3, summer_tts_topicCallBack);
	ros::spin();

	return 0;
}
