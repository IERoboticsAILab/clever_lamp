import cv2
import mediapipe as mp
import torch
import cv2
import numpy as np
import os
import googleapiclient.discovery
from IPython.display import YouTubeVideo, display
import webbrowser
from dotenv import load_dotenv
import google.generativeai as genai
import gtts
from playsound import playsound
import whisper
import sounddevice as sd

load_dotenv()
YOUTUBE_API = os.environ.get("YOUTUBE_DATA_APY_KEY")
youtube = googleapiclient.discovery.build("youtube", "v3", developerKey=YOUTUBE_API)
GEMINI_API = os.environ.get("GEMINI_API_KEY")
genai.configure(api_key=GEMINI_API)
model = genai.GenerativeModel("gemini-1.5-pro")

# Load the Whisper model
model_wisper = whisper.load_model("base")


mp_hands = mp.solutions.hands

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.5
)
hands_videos = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.5
)

mp_drawing = mp.solutions.drawing_utils

def alfread_speacks():
    print("Recording...")
    # wisper
    duration=5
    samplerate=16000
    audio = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype=np.float32)
    sd.wait()  # Wait until recording is finished
    audio = audio.flatten().astype(np.float32)
    # Transcribe the audio
    result = model_wisper.transcribe(audio, fp16=False)
    if 'alfred' in result['text'].lower():
        print("Alfred detected, exiting...")
        print(f"The text from the microphone: \n{result['text']}")
        print("Generating response...")

        prompt = "Answare to the following question in one sentence. Call me sir, and be focued on the key points that I can use to improve my knowlege as an engeneer." + result['text']

        # response
        response = model.generate_content(prompt)
        response = response.text
        # make request to google to get synthesis
        tts = gtts.gTTS(response)
        # save the audio file
        tts.save("response.mp3")
        # play the audio file
        playsound("response.mp3")
    else:
        pass
    print("Done")
    return

def detectHandsLandmarks(image, hands, draw=True):
    '''
    This function performs hands landmarks detection on an image.
    Args:
        image:   The input image with prominent hand(s) whose landmarks needs to be detected.
        hands:   The Hands function required to perform the hands landmarks detection.
        draw:    A boolean value that is if set to true the function draws hands landmarks on the output image.
        display: A boolean value that is if set to true the function displays the original input image, and the output
                 image with hands landmarks drawn if it was specified and returns nothing.
    Returns:
        output_image: A copy of input image with the detected hands landmarks drawn if it was specified.
        results:      The output of the hands landmarks detection on the input image.
    '''

    # Create a copy of the input image to draw landmarks on.
    output_image = image.copy()

    # Convert the image from BGR into RGB format.
    imgRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Perform the Hands Landmarks Detection.
    results = hands.process(imgRGB)

    # Check if landmarks are found and are specified to be drawn.
    if results.multi_hand_landmarks and draw:

        # Iterate over the found hands.
        for hand_landmarks in results.multi_hand_landmarks:

            # Draw the hand landmarks on the copy of the input image.
            mp_drawing.draw_landmarks(image = output_image, landmark_list = hand_landmarks,
                                      connections = mp_hands.HAND_CONNECTIONS,
                                      landmark_drawing_spec=mp_drawing.DrawingSpec(color=(255,255,255),
                                                                                   thickness=2, circle_radius=2),
                                      connection_drawing_spec=mp_drawing.DrawingSpec(color=(0,255,0),
                                                                                     thickness=2, circle_radius=2))

    # Return the output image and results of hands landmarks detection.
    return output_image, results




def countFingers(image, results):
    '''
    to check if finger is up, we compare
    with the previus joint in the x, y plane,
    and we see with one is higher
    (for tumb we chech x coordinate, for other finger the y coordinate)
    '''
    '''
    This function will count the number of fingers up for each hand in the image.
    Args:
        image:   The image of the hands on which the fingers counting is required to be performed.
        results: The output of the hands landmarks detection performed on the image of the hands.
        draw:    A boolean value that is if set to true the function writes the total count of fingers of the hands on the
                 output image.
        display: A boolean value that is if set to true the function displays the resultant image and returns nothing.
    Returns:
        output_image:     A copy of the input image with the fingers count written, if it was specified.
        fingers_statuses: A dictionary containing the status (i.e., open or close) of each finger of both hands.
        count:            A dictionary containing the count of the fingers that are up, of both hands.
    '''

    # Get the height and width of the input image.
    height, width, _ = image.shape

    # Create a copy of the input image to write the count of fingers on.
    output_image = image.copy()

    # Initialize a dictionary to store the count of fingers of both hands.
    count = {'RIGHT': 0, 'LEFT': 0}

    # Store the indexes of the tips landmarks of each finger of a hand in a list.
    fingers_tips_ids = [mp_hands.HandLandmark.INDEX_FINGER_TIP, mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
                        mp_hands.HandLandmark.RING_FINGER_TIP, mp_hands.HandLandmark.PINKY_TIP]

    # Initialize a dictionary to store the status (i.e., True for open and False for close) of each finger of both hands.
    fingers_statuses = {'RIGHT_THUMB': False, 'RIGHT_INDEX': False, 'RIGHT_MIDDLE': False, 'RIGHT_RING': False,
                        'RIGHT_PINKY': False, 'LEFT_THUMB': False, 'LEFT_INDEX': False, 'LEFT_MIDDLE': False,
                        'LEFT_RING': False, 'LEFT_PINKY': False}


    # Iterate over the found hands in the image.
    for hand_index, hand_info in enumerate(results.multi_handedness):

        # Retrieve the label of the found hand.
        hand_label = hand_info.classification[0].label

        # Retrieve the landmarks of the found hand.
        hand_landmarks =  results.multi_hand_landmarks[hand_index]

        # Iterate over the indexes of the tips landmarks of each finger of the hand.
        for tip_index in fingers_tips_ids:

            # Retrieve the label (i.e., index, middle, etc.) of the finger on which we are iterating upon.
            finger_name = tip_index.name.split("_")[0]

            # Check if the finger is up by comparing the y-coordinates of the tip and pip landmarks.
            if (hand_landmarks.landmark[tip_index].y < hand_landmarks.landmark[tip_index - 2].y):

                # Update the status of the finger in the dictionary to true.
                fingers_statuses[hand_label.upper()+"_"+finger_name] = True

                # Increment the count of the fingers up of the hand by 1.
                count[hand_label.upper()] += 1

        # Retrieve the y-coordinates of the tip and mcp landmarks of the thumb of the hand.
        thumb_tip_x = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x
        thumb_mcp_x = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP - 2].x

        # Check if the thumb is up by comparing the hand label and the x-coordinates of the retrieved landmarks.
        if (hand_label=='Right' and (thumb_tip_x < thumb_mcp_x)) or (hand_label=='Left' and (thumb_tip_x > thumb_mcp_x)):

            # Update the status of the thumb in the dictionary to true.
            fingers_statuses[hand_label.upper()+"_THUMB"] = True

            # Increment the count of the fingers up of the hand by 1.
            count[hand_label.upper()] += 1

    # Return the output image, the status of each finger and the count of the fingers up of both hands.
    return output_image, fingers_statuses, count



''' functions for YouTube search '''
class Search_Response:
    def __init__(self, search_response) -> None:
        self.prev_page_token = search_response.get('prevPageToken')
        self.next_page_token = search_response.get('nextPageToken')

        # items element contains a list of videos
        items = search_response.get('items')
        self.search_results = [Search_Result(item) for item in items]

class Search_Result:
    def __init__(self, search_result) -> None:
        self.video_id = search_result['id']['videoId']
        self.title = search_result['snippet']['title']
        self.description = search_result['snippet']['description']
        self.url = f"https://www.youtube.com/watch?v={self.video_id}"
        self.thumbnails = search_result['snippet']['thumbnails']['default']['url']

def search_yt(query, max_results=5, page_token=None):
    request = youtube.search().list(
        part="snippet",
        maxResults=max_results,
        pageToken=page_token,
        q=query,
        videoCaption='closedCaption',
        type='video',
    )
    response = request.execute()
    return Search_Response(response)

# Display YouTube search results and open the first result in Chrome
def display_yt_results(search_response):
    for i, search_result in enumerate(search_response.search_results):
        #print(f'Video ID: {search_result.video_id}')
        #print(f'Title: {search_result.title}')
        #print(f'URL: {search_result.url}')
        youtube_video = YouTubeVideo(search_result.video_id)
        #display(youtube_video)
        #print()

        # Open the first video in Chrome
        if i == 0:
            chrome_path = '/usr/bin/google-chrome-stable %s'  # Adjust if Chrome is installed in a different path
            webbrowser.get(chrome_path).open(search_result.url)