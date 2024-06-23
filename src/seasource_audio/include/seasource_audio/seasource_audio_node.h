//
// Created by lemezoth on 06/06/23.
//

#ifndef BUILD_SEASOURCE_AUDIO_NODE_H
#define BUILD_SEASOURCE_AUDIO_NODE_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "std_msgs/msg/string.hpp"
#include <SDL2/SDL.h>
#include <SDL2/SDL_mixer.h>
#include "std_srvs/srv/set_bool.hpp"
#include <deque>
#include <utility>
#include "seasource_audio/msg/log_audio_source.hpp"
#include "seasource_audio/srv/file_selection.hpp"
#include "seasource_audio/srv/parameters_update.hpp"

using namespace std::chrono_literals;

class SeasourceAudioNode : public rclcpp::Node {
public:

    /**
     * Constructor
     */
    SeasourceAudioNode();

    /**
     * Destructor
     */
    ~SeasourceAudioNode();

private:
    // rclcpp::CallbackGroup::SharedPtr client_cb_group_;
//    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
//    rclcpp::CallbackGroup::SharedPtr service_cb_group_;

public:

    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds sound_duration_between_play_ = 10000ms; // ms
    std::chrono::milliseconds sound_phase_shift_from_posix_ = 0ms; // ms

    /// SDL
    SDL_AudioSpec wavSpec_;
    SDL_AudioDeviceID audioDevice_;
    int sound_volume_ = 128; // Between 0 and 128

    std::vector<Mix_Chunk*> music_; // The music file to play
    bool music_loaded_ = false;

    std::string path_data_audio_ = "";
    std::vector<std::string> audio_files_;

    size_t current_audio_file_ = 0;
    size_t audio_file_id_default_ = 1;

    enum class AudioMode {
        SINGLE_FILE=1,
        SEQUENTIAL=2,
    };
    AudioMode audio_mode_ = AudioMode::SINGLE_FILE;

    bool enable_play_ = true;

    /// Interfaces
    rclcpp::Publisher<seasource_audio::msg::LogAudioSource>::SharedPtr publisher_log_audio_source_;
    rclcpp::Service<seasource_audio::srv::FileSelection>::SharedPtr service_file_selection_;
    rclcpp::Service<seasource_audio::srv::ParametersUpdate>::SharedPtr service_parameters_update_;

    /// Parameters

    /// Functions

    /**
     * Play audio
     */
    void play_audio() const;

    /**
     * Load music file
     */
    void load_music();

    /**
     * Init SDL
     */
    void init_SDL();

    /**
     *  Init and get parameters of the Node
     */
    void init_parameters();

    /**
     * Init interfaces to this node (publishers & subscribers)
     */
    void init_interfaces();

    /**
     * timer callback
     */
    void timer_callback();

    /**
     * Load audio files
     */
    void load_audio_files();

    void callback_file_selection(const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<seasource_audio::srv::FileSelection::Request> request,
                                 const std::shared_ptr<seasource_audio::srv::FileSelection::Response> response);

    void callback_parameters_update(const std::shared_ptr<rmw_request_id_t> request_header,
                                    const std::shared_ptr<seasource_audio::srv::ParametersUpdate::Request> request,
                                    const std::shared_ptr<seasource_audio::srv::ParametersUpdate::Response> response);

    void sync_timer_start();
};

#endif //BUILD_SEASOURCE_AUDIO_NODE_H
