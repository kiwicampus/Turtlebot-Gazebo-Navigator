/*! @package speaker
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#include "modules/speaker.hpp"

Speaker::Speaker(rclcpp::NodeOptions &options) : Node("speaker", "interfaces", options)
{
    RCLCPP_DEBUG(this->get_logger(), "Speaker Contructor");

    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    // /* Publisher */
    m_done_pub = this->create_publisher<std_msgs::msg::Bool>("/device/speaker/done", default_qos);

    // Subscribers
    m_speaker_sub = this->create_subscription<std_msgs::msg::String>(
                "/device/speaker/command", default_qos, std::bind(&Speaker::speakerCb, this, _1));

    /* Open the PCM device in playback mode */
    if (pcm = (snd_pcm_open(&pcm_handle, m_sound_device.c_str(), SND_PCM_STREAM_PLAYBACK, 0) < 0))
        RCLCPP_ERROR(this->get_logger(), "ERROR: Can't open \"%s\" PCM device. %s", m_sound_device, snd_strerror(pcm));

    /* Allocate parameters object and fill it with default values*/
    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(pcm_handle, params);

    /* Set parameters */
    if (pcm = (snd_pcm_hw_params_set_access(pcm_handle, params, SND_PCM_ACCESS_RW_INTERLEAVED) < 0))
        RCLCPP_ERROR(this->get_logger(), "ERROR: Can't set interleaved mode. %s", snd_strerror(pcm));

    if (pcm = (snd_pcm_hw_params_set_format(pcm_handle, params, SND_PCM_FORMAT_S16_LE) < 0))

        RCLCPP_ERROR(this->get_logger(), "ERROR: Can't set format. %s\n", snd_strerror(pcm));

    if (pcm = (snd_pcm_hw_params_set_channels(pcm_handle, params, channels) < 0))
        RCLCPP_ERROR(this->get_logger(), "ERROR: Can't set channels number. %s\n", snd_strerror(pcm));

    if (pcm = (snd_pcm_hw_params_set_rate_near(pcm_handle, params, &rate, 0) < 0))
        RCLCPP_ERROR(this->get_logger(), "ERROR: Can't set rate. %s\n", snd_strerror(pcm));

    /* Write parameters */
    if (pcm = (snd_pcm_hw_params(pcm_handle, params) < 0))
        RCLCPP_ERROR(this->get_logger(), "ERROR: Can't set harware parameters. %s\n", snd_strerror(pcm));

    /* Resume information */
    RCLCPP_INFO(this->get_logger(), "PCM name: '%s'\n", snd_pcm_name(pcm_handle));

    RCLCPP_INFO(this->get_logger(), "PCM state: %s\n", snd_pcm_state_name(snd_pcm_state(pcm_handle)));

    snd_pcm_hw_params_get_period_size(params, &frames, 0);

    buff_size = frames * channels * 2;
    buff = (char *)malloc(buff_size);
    memset(buff, 0, buff_size);

    m_path = "/workspace/media/audio/";

    snd_pcm_start(pcm_handle);
}

void Speaker::speakerCb(const std_msgs::msg::String::SharedPtr msg)
{
    /*
+       Note: Every sound-track should be named "track#"
        To Stop send a 0 message.
    */
    m_multi_sound = 0;
    snd_pcm_start(pcm_handle);
    if (!msg->data.empty())
    {
        std::ifstream ifile;
        ifile.open(m_path + msg->data);
        if (ifile)
        {
            readfd = open((m_path + msg->data).c_str(), O_RDONLY);
            status = pthread_create(&pthread_id, NULL, (THREADFUNCPTR)&Speaker::PlaySound, this);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Track %s not found", msg->data.c_str());
        }
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "Sound stopped");
        m_pause = 1;
        m_multi_sound = 1;
    }
}

void *Speaker::PlaySound()
{
    if (readfd < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "ERROR: .wav It's Empty");
        pthread_join(pthread_id, NULL);
    }
    std_msgs::msg::Bool::UniquePtr msg(new std_msgs::msg::Bool());
    msg->data = false;
    m_done_pub->publish(std::move(msg));


    while (readval = (read(readfd, buff, buff_size) > 0))
    {
        if (pcm = snd_pcm_writei(pcm_handle, buff, frames) == -EPIPE)
        {
            snd_pcm_prepare(pcm_handle);
        }
        else if (pcm < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Writing to PCM device: %s", snd_strerror(pcm));
        }
    }
    m_multi_sound = 1;

    msg.reset(new std_msgs::msg::Bool());

    msg->data = true;
    m_done_pub->publish(std::move(msg));

}  

/* Class Destructor to clean up everything*/
Speaker::~Speaker()
{
    pthread_kill(pthread_id, 15);
    free(buff);
    snd_pcm_close(pcm_handle);
}