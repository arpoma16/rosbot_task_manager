#!/usr/bin/env python3
import rospy
import speech_recognition as sr
from speech_recognition.srv import Command_service

import get_yalm



def recognize_speech_from_mic(recognizer, microphone):
    """Transcribe speech from recorded from `microphone`.

    Returns a dictionary with three keys:
    "success": a boolean indicating whether or not the API request was
               successful
    "error":   `None` if no error occured, otherwise a string containing
               an error message if the API could not be reached or
               speech was unrecognizable
    "transcription": `None` if speech could not be transcribed,
               otherwise a string containing the transcribed text
    """
    # check that recognizer and microphone arguments are appropriate type
    if not isinstance(recognizer, sr.Recognizer):
        raise TypeError("`recognizer` must be `Recognizer` instance")

    if not isinstance(microphone, sr.Microphone):
        raise TypeError("`microphone` must be `Microphone` instance")

    # adjust the recognizer sensitivity to ambient noise and record audio
    # from the microphone
    with microphone as source:
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

    # set up the response object
    response = {
        "success": True,
        "error": None,
        "transcription": None
    }

    # try recognizing the speech in the recording
    # if a RequestError or UnknownValueError exception is caught,
    #     update the response object accordingly
    try:
        response["transcription"] = recognizer.recognize_google(audio,language='es-ES')
    except sr.RequestError:
        # API was unreachable or unresponsive
        response["success"] = False
        response["error"] = "API unavailable"
    except sr.UnknownValueError:
        # speech was unintelligible
        response["error"] = "Unable to recognize speech"

    return response

def Send_command(command_recognice):
    rospy.wait_for_service('Service_command')
    try:
        send_commad_srv = rospy.ServiceProxy('Service_command', Command_service)
        resp1 = send_commad_srv(command_recognice)
        return resp1.c
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



if __name__ == "__main__":
    rospy.init_node('Service_command_server')
    path_yalm = rospy.get_param("yalm_path")
    list_command =  get_yalm.get_command(path_yalm)

    # create recognizer and mic instances
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()

    while True:

        guess = recognize_speech_from_mic(recognizer, microphone)
        rospy.INFO("voice command detected: " + guess["transcription"])

        if guess["error"]:
            print("ERROR: {}".format(guess["error"]))
            break
        if "cancelar" in guess["transcription"].lower():
            rospy.INFO("comando reconocido: cancelar")
            Send_command("cancelar")

        for commands in list_command["commmand"]:
            if  commands["name"] in guess["transcription"].lower():
                rospy.INFO("comando reconocido: " + commands["name"])
                Send_command(commands["name"])
                break
            

        rospy.sleep(0.5)
