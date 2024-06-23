//
// Created by lemezoth on 06/06/23.
//

#include "seasource_audio/seasource_audio_node.h"
#include <iostream>
#include <fmt/core.h>
#include <filesystem>
#include <fstream>
#include <pwd.h>
#include <unistd.h>
#include <sys/stat.h>

using namespace std;
using namespace std::placeholders;

SeasourceAudioNode::SeasourceAudioNode()
        : Node("seasource_audio"){

    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Default value

    init_parameters();
    init_interfaces();
    init_SDL();

    load_audio_files();

    timer_ = this->create_wall_timer(
            sound_duration_between_play_, std::bind(&SeasourceAudioNode::timer_callback, this), timer_cb_group_);
    timer_->cancel();

    // Wait for the next phase shift
    auto now = std::chrono::system_clock::now();
    auto duration_since_epoch = now.time_since_epoch();
    auto duration_modulo = duration_since_epoch % sound_duration_between_play_;
    auto duration_to_sleep = sound_duration_between_play_ - duration_modulo + sound_phase_shift_from_posix_;

    // Sleep until the next phase shift
    std::this_thread::sleep_for(duration_to_sleep);
    timer_->reset();
    timer_->execute_callback();

    RCLCPP_INFO(this->get_logger(), "[seasource_audio_node] Start Ok");
}

SeasourceAudioNode::~SeasourceAudioNode() {
    if (music_) {
        Mix_FreeChunk(music_);
    }
    Mix_CloseAudio();
    SDL_Quit();
}

void SeasourceAudioNode::load_audio_files(){
    auto exePath = filesystem::canonical("/proc/self/exe");

    path_data_audio_ = std::filesystem::absolute(
            exePath.parent_path().append("data/"))
            .string();

    RCLCPP_INFO(this->get_logger(), "[seasource_audio_node] path_data_audio_: %s", path_data_audio_.c_str());

    // Load all audio files in alphabetical order and store the filename in a vector
    for (const auto &entry : filesystem::directory_iterator(path_data_audio_)) {
        if (entry.is_regular_file()) {
            audio_files_.push_back(entry.path().filename().string());
        }
    }

    // Reorder the audio files base on the first number in the filename
    std::sort(audio_files_.begin(), audio_files_.end(), [](const std::string &a, const std::string &b) {
        return stoi(a) < stoi(b);
    });

    // Display the audio files
    for (const auto &audio_file : audio_files_) {
        RCLCPP_INFO(this->get_logger(), "[seasource_audio_node] audio_file: %s", audio_file.c_str());
    }


    if (audio_files_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "[seasource_audio_node] No audio files found in %s", path_data_audio_.c_str());
        return;
    }

    load_music(audio_files_[current_audio_file_]);
}

void SeasourceAudioNode::init_parameters() {
    this->declare_parameter<long>("sound_duration_between_play", sound_duration_between_play_.count());
    sound_duration_between_play_ = std::chrono::milliseconds(this->get_parameter_or("sound_duration_between_play", sound_duration_between_play_.count()));

    this->declare_parameter<long>("sound_phase_shift_from_posix", sound_phase_shift_from_posix_.count());
    sound_phase_shift_from_posix_ = std::chrono::milliseconds(this->get_parameter_or("sound_phase_shift_from_posix", sound_phase_shift_from_posix_.count()));

    // Verify that sound_phase_shift_from_posix is less than sound_duration_between_play
    if (sound_phase_shift_from_posix_ >= sound_duration_between_play_) {
        RCLCPP_ERROR(this->get_logger(), "[seasource_audio_node] sound_phase_shift_from_posix must be less than sound_duration_between_play");
    }

    this->declare_parameter<int>("audio_file_id", current_audio_file_);
    current_audio_file_ = this->get_parameter_or("audio_file_id", current_audio_file_);

    RCLCPP_INFO(this->get_logger(), "[seasource_audio_node] init_parameters done");
}

void SeasourceAudioNode::init_SDL(){
    // Initialize SDL
    if (SDL_Init(SDL_INIT_AUDIO) < 0) {
        std::cerr << "SDL initialization failed: " << SDL_GetError() << std::endl;
        exit(EXIT_FAILURE);
    }

    int count = SDL_GetNumAudioDevices(0);
    for (int i = 0; i < count; ++i)
    {
        std::cout << "Device " << i << ": " << SDL_GetAudioDeviceName(i, 0) << std::endl;
    }

    // Initialize SDL2_mixer
    if (Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 2048) < 0) {
        printf("SDL_mixer could not initialize! SDL_mixer Error: %s\n", Mix_GetError());
        exit(EXIT_FAILURE);
    }

    //
    Mix_VolumeMusic(sound_volume_);

    // Call amixer command to set the volume
    string command = "amixer -D pulse sset Master unmute";
    system(command.c_str());

    command = "amixer -D pulse sset Master 100%";
    system(command.c_str());

    RCLCPP_INFO(this->get_logger(), "[seasource_audio_node] init_SDL done");
}

void SeasourceAudioNode::timer_callback(){
    play_audio();
}

void SeasourceAudioNode::load_music(const std::string &audio_id) {
    string file = path_data_audio_ + audio_id;

    // Load wav file
    music_ = Mix_LoadWAV(file.c_str());
    if (!music_) {
        printf("Failed to load music! SDL_mixer Error: %s\n", Mix_GetError());
        return;
    }
    else{
        music_loaded_ = true;
    }
}

void SeasourceAudioNode::play_audio() const {
    if (!music_loaded_)
        return;

    // Play the music
    Mix_PlayChannel(-1, music_, 0);

    seasource_audio::msg::LogAudioSource msg;
    msg.header.stamp = this->now();
    msg.filename = audio_files_[current_audio_file_];
    msg.file_id = current_audio_file_;
    publisher_log_audio_source_->publish(msg);

    // Wait for music to finish
    while (Mix_PlayingMusic()) {
        SDL_Delay(100);
    }
}

void SeasourceAudioNode::init_interfaces() {
    // Publisher
    publisher_log_audio_source_ = this->create_publisher<seasource_audio::msg::LogAudioSource>(
            "log_audio_source", 10);

    RCLCPP_INFO(this->get_logger(), "[seasource_audio_node] init_interfaces done");
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SeasourceAudioNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
