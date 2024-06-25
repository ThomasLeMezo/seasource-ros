import numpy as np
import wave
import struct

wave_id = 1

def generate_sine_wave(frequency, duration, sample_rate=192000, amplitude=32767):
    t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
    wave = amplitude * np.sin(2 * np.pi * frequency * t)
    return wave.astype(np.int32)

def generate_chirp_wave(start_freq, end_freq, duration, sample_rate=192000, amplitude=32767):
    t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
    wave = amplitude * np.sin(2 * np.pi * ((start_freq * t) + ((end_freq - start_freq) / (2 * duration)) * t**2))
    return wave.astype(np.int32)

def generate_multitone_wave(frequencies, duration, sample_rate=192000, amplitude=32767):
    t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
    wave = np.zeros_like(t)
    for frequency in frequencies:
        wave += amplitude * np.sin(2 * np.pi * frequency * t)
    # Normalize the wave to prevent clipping
    wave = np.clip(wave, -amplitude, amplitude)
    return wave.astype(np.int32)

def save_wave(filename, wave_data, sample_rate=192000, export_folder='../data'):
    global wave_id

    # convert wave_id to str with zero padding
    wave_id_str = str(wave_id).zfill(4) + "_"

    complete_filename = export_folder + "/" + wave_id_str + filename
    with wave.open(complete_filename, 'w') as wf:
        wf.setnchannels(2)  # mono
        wf.setsampwidth(4)  # 16-bit samples
        wf.setframerate(sample_rate)
        wf.writeframes(wave_data.tobytes())

        print(f"Wave file {complete_filename} saved successfully.")
        wave_id += 1

def main():
    # export folder for the generated audio files
    export_folder = '/home/source/audio'

    sample_rate = 192000
    # Amplitude of the wave (32-bit signed integer)
    amplitude = 480761704

    # Sine wave parameters
    frequency = 440.0  # Frequency in Hz (e.g., A4 note)
    duration = 2.0     # Duration in seconds
    sine_wave = generate_sine_wave(frequency, duration, sample_rate, amplitude)
    save_wave('sine_wave.wav', sine_wave, sample_rate, export_folder)

    # Chirp wave parameters
    start_freq = 200.0  # Starting frequency of the chirp
    end_freq = 800.0    # Ending frequency of the chirp
    duration = 1.0     # Duration in seconds
    chirp_wave = generate_chirp_wave(start_freq, end_freq, duration, sample_rate, amplitude)
    save_wave('chirp_wave.wav', chirp_wave, sample_rate, export_folder)

    # Multitone wave parameters
    frequencies = [440.0, 550.0, 660.0]  # List of frequencies for the multitone wave
    duration = 2.0     # Duration in seconds
    multitone_wave = generate_multitone_wave(frequencies, duration, sample_rate, amplitude)
    save_wave('multitone_wave.wav', multitone_wave, sample_rate, export_folder)

if __name__ == "__main__":
    main()

