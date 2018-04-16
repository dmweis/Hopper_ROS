#!/usr/bin/env python

from __future__ import print_function
import json
from google.cloud import texttospeech
import os.path

files_to_syntetize = json.load(open("soundFiles.json"))
client = texttospeech.TextToSpeechClient()

voice = texttospeech.types.VoiceSelectionParams(
    language_code='en-US',
    ssml_gender=texttospeech.enums.SsmlVoiceGender.FEMALE)

# Select the type of audio file you want returned
audio_config = texttospeech.types.AudioConfig(
    audio_encoding=texttospeech.enums.AudioEncoding.OGG_OPUS)

for recording in files_to_syntetize:
    file_path = "output\\ {0}.ogg".format(recording["name"])
    if os.path.exists(file_path):
        print("File: {0} already exists. If you'd like topdate it, delete the original file".format(recording["name"]))
        continue
    synthesis_input = None
    if "text" in recording:
        synthesis_input = texttospeech.types.SynthesisInput(text=recording["text"])
    elif "ssml" in recording:
        synthesis_input = texttospeech.types.SynthesisInput(ssml=recording["ssml"])
    else:
        print("File: {0} is missing text and ssml field".format(recording["name"]))
        continue
    response = client.synthesize_speech(synthesis_input, voice, audio_config)
    with open(file_path, 'wb') as out:
        out.write(response.audio_content)
        print(recording["name"] + " was successfully written")
