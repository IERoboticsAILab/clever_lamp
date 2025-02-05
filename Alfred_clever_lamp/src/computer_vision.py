#!/usr/bin/env python3

import sys
import os
import cv2
from PIL import Image as PIL_Image
import google.generativeai as genai
import mediapipe as mp
import torch
import numpy as np
from dotenv import load_dotenv
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '../scripts'))
from my_functions import alfread_speacks, countFingers, detectHandsLandmarks, Search_Response, Search_Result, search_yt, display_yt_results
import webbrowser
import googleapiclient.discovery
from IPython.display import YouTubeVideo, display
import whisper
import sounddevice as sd
from playsound import playsound
import gtts
import pychromecast
import zeroconf
from pychromecast.controllers.youtube import YouTubeController
#import threading
#from queue import Queue
#import pyautogui # pip install pyautogui # --> for moving in the browser



''' SET UP HANDS MODULE '''
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False,max_num_hands=2,min_detection_confidence=0.5)
hands_videos = mp_hands.Hands(static_image_mode=False,max_num_hands=2,min_detection_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils


''' SET UP YOUTUBE '''
zconf = zeroconf.Zeroconf()
chromecasts, browser = pychromecast.get_listed_chromecasts(friendly_names=["Ufficio"])
if not chromecasts:
    print("\n\n\n\nNo Chromecast with the given name found.\n\n\n\n")
    exit()
cast = chromecasts[0]
cast.wait()
print(f"Connected to: {cast.cast_info.friendly_name}")
yt = YouTubeController()
cast.register_handler(yt)
#yt.launch()


''' SET UP GEMINI MODULE '''
load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), "../scripts/.env"))
GEMINI_API = os.environ.get("GEMINI_API_KEY")
genai.configure(api_key=GEMINI_API)
model = genai.GenerativeModel("gemini-1.5-pro")


''' SET UP WHISPER MODULE '''
model_wisper = whisper.load_model("base")


''' SET UP WEBCAM'''
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 500)


''' SET UP YOUTUBE API '''
YOUTUBE_API = os.environ.get("YOUTUBE_DATA_APY_KEY")
youtube = googleapiclient.discovery.build("youtube", "v3", developerKey=YOUTUBE_API)


''' HANDS GESTURE RECOGNITION '''
def recognizeGestures(image, fingers_statuses, count):
    output_image = image.copy()
    hands_labels = ['RIGHT', 'LEFT']
    hands_gestures = {'RIGHT': "UNKNOWN", 'LEFT': "UNKNOWN"}
    for hand_index, hand_label in enumerate(hands_labels):
        if count[hand_label] == 2  and fingers_statuses[hand_label+'_MIDDLE'] and fingers_statuses[hand_label+'_INDEX']:
            hands_gestures[hand_label] = "V SIGN"
        elif count[hand_label] == 3 and fingers_statuses[hand_label+'_THUMB'] and fingers_statuses[hand_label+'_INDEX'] and fingers_statuses[hand_label+'_PINKY']:
            hands_gestures[hand_label] = "SPIDERMAN SIGN"
        elif count[hand_label] == 5:
            hands_gestures[hand_label] = "HIGH-FIVE SIGN"
        elif (count[hand_label] == 1 and fingers_statuses[hand_label+'_INDEX']) or (count[hand_label] == 2 and fingers_statuses[hand_label+'_INDEX'] and fingers_statuses[hand_label+'_THUMB']):
            hands_gestures[hand_label] = "POINTING"
            screenshot_path = "pointing_object.jpg"
            cv2.imwrite(screenshot_path, frame)
    return output_image, hands_gestures


''' INITIALIZE VARIABLES '''
pointing_detected_frames = 0    # -> Counts consecutive frames of pointing detection
pointing_stable_threshold = 15  # -> Number of frames to consider as stable
cooldown_counter = 0            # -> Counter for cooldown
cooldown_frames = 30            # -> Cooldown period in frames
is_processing = False           # -> Flag to indicate if processing is ongoing
speack_back = False              # -> to make Alfred speack back


''' MAIN LOOP '''
while cap.isOpened():
    ret, frame = cap.read()
    frame = cv2.flip(frame, -1)
    if not ret:
        break

    ''' if not processing and cooldown is complete, proceed with detection '''
    if not is_processing and cooldown_counter == 0:
        output_image, results = detectHandsLandmarks(frame, hands, draw=True)
        cv2.imshow('Webcam with Hand Landmarks', output_image)

        ''' count fingers and detect gestures '''
        if results.multi_hand_landmarks:
            output_image, fingers_statuses, count = countFingers(frame, results)
            output_image, hands_gestures = recognizeGestures(output_image, fingers_statuses, count)
            #print(count)
            #print(fingers_statuses)
            #print(hands_gestures)

            ''' check for pointing gesture reached stable threshold '''
            if "POINTING" in hands_gestures.values():
                pointing_detected_frames += 1
                print(f"Pointing gesture detected for {pointing_detected_frames} frames.")
            else:
                pointing_detected_frames = 0

            ''' if pointing gesture is stable, proceed with processing '''
            if pointing_detected_frames >= pointing_stable_threshold:
                is_processing = True

                ''' save image for processing '''
                screenshot_path = "pointing_object.jpg"
                cv2.imwrite(screenshot_path, frame)

                ''' Generate links and description using GEMINI '''
                image = cv2.imread(screenshot_path)
                image = PIL_Image.fromarray(image)
                cv2.waitKey(2000)

                ''' generate links and description using GEMINI '''
                prompt = """
                    I need a very careful structured response from you:
                        - Describe what I am pointing at with my index finger in exactly two words.
                        - Then, give me a brief description of the object in one sentence.
                        - Then give me exactly one Wikipedia link to dive deeper into the topic.
                """
                contents = [image, prompt]
                print("\n-------thinking--------")
                response = model.generate_content(contents)
                response_text = response.text
                print("\n-------Response--------")
                print(response_text)

                ''' open wikipedia url in GEMINI response '''
                start_index = response_text.index('[')
                end_index = response_text.index(']')
                url = response_text[start_index+1:end_index]
                # open usrl on firefox
                webbrowser.get('firefox').open_new_tab(url)
                #webbrowser.open_new_tab(url)
                ''' open youtube url using GEMINI 2 words description '''
                start_index = response_text.index('\n')
                print(f"\n{response_text[:start_index]}")
                search_response = search_yt(response_text[:start_index])
                for i, search_result in enumerate(search_response.search_results):
                    print(f"video id {i}: {search_result.video_id}")
                    if i == 0:
                        yt.play_video(search_result.video_id)
                        print(f"Playing YouTube video: {search_result.video_id}")
                        break
                ''' make Alfred talk back '''
                #if speack_back:
                #    alfread_speacks()

                ''' reset stability counter and cooldown counter '''
                pointing_detected_frames = 0
                cooldown_counter = cooldown_frames
                is_processing = False
                cv2.waitKey(2000)

    ''' if no hands detected, reset stability counter '''
    if cooldown_counter > 0:
        cooldown_counter -= 1

    ''' break the loop if 'q' is pressed '''
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

''' release resources '''
cap.release()
cv2.destroyAllWindows()
