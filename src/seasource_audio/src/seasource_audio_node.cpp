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

//    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
//    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Default value

    init_parameters();
    init_interfaces();
    init_SDL();

    load_audio_files();

    sync_timer_start();

    RCLCPP_INFO(this->get_logger(), "[seasource_audio_node] Start Ok");
}

SeasourceAudioNode::~SeasourceAudioNode() {
    Mix_CloseAudio();
    SDL_Quit();
}

void SeasourceAudioNode::sync_timer_start() {
    if(timer_)
        timer_->cancel();

    // Change timer duration
    timer_ = this->create_wall_timer(
            sound_duration_between_play_, std::bind(&SeasourceAudioNode::timer_callback, this));
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

    load_music();
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

    size_t audio_file_id = audio_file_id_default_;
    this->declare_parameter<int>("audio_file_id", (int)audio_file_id);
    audio_file_id = this->get_parameter_or("audio_file_id", audio_file_id);

    if(audio_file_id == 0){
        current_audio_file_ = 0;
        audio_mode_ = AudioMode::SEQUENTIAL;
        RCLCPP_INFO(this->get_logger(), "[seasource_audio_node] audio_mode: SEQUENTIAL");
    }
    else{
        current_audio_file_ = audio_file_id-1;
        audio_mode_ = AudioMode::SINGLE_FILE;
        RCLCPP_INFO(this->get_logger(), "[seasource_audio_node] audio_mode: SINGLE_FILE");
    }

    RCLCPP_INFO(this->get_logger(), "[seasource_audio_node] audio_file_id: %d", current_audio_file_);

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
    if (Mix_OpenAudio(192000, AUDIO_S32LSB, 2, 2048) < 0) {
        printf("SDL_mixer could not initialize! SDL_mixer Error: %s\n", Mix_GetError());
        exit(EXIT_FAILURE);
    }

    //
    Mix_VolumeMusic(sound_volume_);

    // Call amixer command to set the volume
    string command = "amixer -D pulse sset Master unmute";
    system(command.c_str());

    command = "amixer -D pulse sset Master 20%";
    system(command.c_str());

    RCLCPP_INFO(this->get_logger(), "[seasource_audio_node] init_SDL done");
}

void SeasourceAudioNode::timer_callback(){
    play_audio();

    if(audio_mode_ == AudioMode::SEQUENTIAL){
        current_audio_file_ = (current_audio_file_ + 1) % music_.size();
    }
}

void SeasourceAudioNode::load_music() {
    for(const auto &audio_file : audio_files_){
        music_.push_back(nullptr);
    }

    for(size_t index = 0; index<audio_files_.size(); index++){
        string file = path_data_audio_ + audio_files_[index];

        // Test if file exists
        if (!filesystem::exists(file)) {
            RCLCPP_ERROR(this->get_logger(), "[seasource_audio_node] File %s does not exist", file.c_str());
            return;
        }

        // Free the previous music
        if (music_[index]) {
            Mix_FreeChunk(music_[index]);
        }

        // Load wav file
        music_[index] = Mix_LoadWAV(file.c_str());
        if (!music_[index]) {
            RCLCPP_ERROR(this->get_logger(), "[seasource_audio_node] Mix_LoadWAV: %s", Mix_GetError());
        }
    }
}

void SeasourceAudioNode::play_audio() const {
    // Play the music
    if(Mix_PlayChannel(-1, music_[current_audio_file_], 0) == -1) {
        RCLCPP_ERROR(this->get_logger(), "[seasource_audio_node] Mix_PlayChannel: %s", Mix_GetError());
        return;
    }

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

    // Service
    service_file_selection_ = this->create_service<seasource_audio::srv::FileSelection>(
            "file_selection",
            std::bind(&SeasourceAudioNode::callback_file_selection, this, _1, _2, _3),
            rmw_qos_profile_services_default);

    service_parameters_update_ = this->create_service<seasource_audio::srv::ParametersUpdate>(
            "parameters_update",
            std::bind(&SeasourceAudioNode::callback_parameters_update, this, _1, _2, _3),
            rmw_qos_profile_services_default);

    RCLCPP_INFO(this->get_logger(), "[seasource_audio_node] init_interfaces done");
}

void SeasourceAudioNode::callback_file_selection(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<seasource_audio::srv::FileSelection::Request> request,
        const std::shared_ptr<seasource_audio::srv::FileSelection::Response> response) {
    (void)request_header;

    if(request->audio_file_id < 0 || request->audio_file_id >= (int)music_.size()){
        RCLCPP_ERROR(this->get_logger(), "[seasource_audio_node] File id %d is out of range", request->audio_file_id);
        response->success = false;
        return;
    }

    current_audio_file_ = request->audio_file_id;
    response->success = true;
}

void SeasourceAudioNode::callback_parameters_update(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<seasource_audio::srv::ParametersUpdate::Request> request,
        const std::shared_ptr<seasource_audio::srv::ParametersUpdate::Response> response) {
    (void)request_header;

    if(request->sound_duration_between_play > 0){
        sound_duration_between_play_ = std::chrono::milliseconds(request->sound_duration_between_play);
    }

    if(request->sound_phase_shift_from_posix >= 0){
        sound_phase_shift_from_posix_ = std::chrono::milliseconds(request->sound_phase_shift_from_posix);
    }

    sync_timer_start();

    response->success = true;
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
