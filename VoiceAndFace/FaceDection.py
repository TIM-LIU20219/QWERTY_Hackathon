import time
from gtts import gTTS
import numpy as np
import speech_recognition as sr
import face_recognition
import cv2
import os
dir_path = os.path.dirname(os.path.realpath(__file__))

stopper = set(['stop', 'finish', 'terminate', 'quit', 'goodbye'])
welcome_text = "Voice control and Face detection"
output = gTTS(welcome_text, lang='en', slow=False)
output.save('welcoeme.mp3')
os.system('mpg123 welcoeme.mp3 ')


def face_monitor():
    video_capture = cv2.VideoCapture(0)

    def get_encoding(name_list):
        encoding = []
        for i, name in enumerate(name_list):
            path = dir_path + '/pictures/' + str(i) + ".jpg"
            img = face_recognition.load_image_file(path)
            encoding.append(face_recognition.face_encodings(img)[0])
        return encoding

    name_list = ['Tim']
    # name_list = ['Tim', 'Jade', 'David', 'Seb', 'Dequan', 'Ti']
    appeared = [False] * len(name_list)
    pic_dir = 'VoiceAndFace/pictures/'
    known_encoding_list = get_encoding(name_list)

    face_locations = []
    face_encodings = []
    face_names = []

    # cnt = 0
    start = time.time()
    new_command = ''
    while True:

        end = time.time()
        ret, frame = video_capture.read()
        # cnt += 1
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        rgb_small_frame = small_frame[:, :, ::-1]

        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(
            rgb_small_frame, face_locations)

        face_names = []

        for face_encoding in face_encodings:

            matches = face_recognition.compare_faces(
                known_encoding_list, face_encoding)
            name = "Unknown"

            face_distances = face_recognition.face_distance(
                known_encoding_list, face_encoding)
            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                name = name_list[best_match_index]
                if not appeared[best_match_index]:
                    # cv2.putText(frame, 'Welcome', (frame.shape[0]/2, frame.shape[1]/2),
                    #             cv2.FONT_HERSHEY_DUPLEX, 1.0, (255, 255, 255), 1)
                    cv2.putText(frame, 'Welcome', (0, 0),
                                cv2.FONT_HERSHEY_DUPLEX, 1.0, (255, 255, 255), 1)
                    print("Welcome")
                    welcome_text2 = "Hello!" + name
                    output = gTTS(welcome_text2, lang='en', slow=False)
                    file_name = 'welcoeme' + name + '.mp3'
                    output.save(file_name)
                    readr_name = 'mpg123 ' + file_name
                    print(readr_name)
                    os.system(readr_name)
                    appeared[best_match_index] = True
            face_names.append(name)

        # Display the results
        for (top, right, bottom, left), name in zip(face_locations, face_names):
            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4

            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

            cv2.rectangle(frame, (left, bottom - 35),
                          (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, name, (left + 6, bottom - 6),
                        font, 1.0, (255, 255, 255), 1)
            if end - start > 1.5:
                # print("detection lapse: ", end - start)
                start = end
                with open(dir_path + '/utils/command.txt', 'r') as f:
                    tmp = f.read().splitlines()
                    cur_command = ""
                    for item in tmp:
                        cur_command += str(item) + " "
                        if str(item) in stopper:
                            print("Voice exit")
                            return
                    new_command = cur_command
                    print(new_command)
            if len(new_command) > 0:
                cv2.putText(frame, new_command, (50, 50),
                            font, 1.3, (0, 255, 255), 1)

        cv2.imshow('Video', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    video_capture.release()
    cv2.destroyAllWindows()


face_monitor()
