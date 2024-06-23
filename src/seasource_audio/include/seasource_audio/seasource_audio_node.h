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
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

public:

    /// Rclcpp
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds sound_duration_between_play_ = 10000ms; // ms
    std::chrono::milliseconds sound_phase_shift_from_posix_ = 0ms; // ms

    /// SDL
    SDL_AudioSpec wavSpec_;
    SDL_AudioDeviceID audioDevice_;
    int sound_volume_ = 128; // Between 0 and 128

    Mix_Chunk* music_ = nullptr; // The music file to play
    bool music_loaded_ = false;

    std::string path_data_audio_ = "";
    std::vector<std::string> audio_files_;

    size_t current_audio_file_ = 0;

    /// Interfaces
    rclcpp::Publisher<seasource_audio::msg::LogAudioSource>::SharedPtr publisher_log_audio_source_;

    /// Parameters

    /// Functions

    /**
     * Play audio
     */
    void play_audio() const;

    /**
     * Load music file
     * @param audio_id
     */
    void load_music(const std::string &audio_id);

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

};

#endif //BUILD_SEASOURCE_AUDIO_NODE_H
