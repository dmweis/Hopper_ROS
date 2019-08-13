#!/usr/bin/env python

from __future__ import print_function
import json
from google.cloud import texttospeech
import os.path

client = texttospeech.TextToSpeechClient()

voice = texttospeech.types.VoiceSelectionParams(
    language_code='en-US',
    ssml_gender=texttospeech.enums.SsmlVoiceGender.FEMALE)

# Select the type of audio file you want returned
audio_config = texttospeech.types.AudioConfig(audio_encoding=texttospeech.enums.AudioEncoding.OGG_OPUS)

synthesis_input = texttospeech.types.SynthesisInput(text="I am hopper. How can I help you?")
response = client.synthesize_speech(synthesis_input, voice, audio_config)
with open("out.wav", 'wb') as out:
    out.write(response.audio_content)
    print("file was successfully written")
