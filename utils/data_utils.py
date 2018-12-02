import numpy as np 
import cv2
import os 
import sys


class Pedestrian():
    def __init__(self):
        self.subject = -1
        self.position_list = []
        self.velocity_list = []
        self.preferred_speed = -1
        self.chosen_destination = None
        self.start_frame = -1
        self.stop_frame = -1
        self.pedestrian_group = []

    def extract_subject_trajectory(self, annotationfile, subject):
        start_flag = False
        self.subject = subject
        if not os.path.isfile(annotationfile):
            print("The annotation file does not exist.")
            return 0

        annotation_list = []
        annotation_dict = {}
        print("Loading annotation file . . .")
        with open(annotationfile) as f:
            for line in f:
                line = line.strip().split(' ')
                # print line
                # print "break"
                annotation_list.append(line)

        # The position is the midpoint of the bounding box
        for i in range(len(annotation_list)-1):  # Skips last frame
            old_pos = (-1, -1)
            listval = annotation_list[i]
            listval_future = annotation_list[i+1]
            speed = 0
            counter = 0
            # get all annotations for this subject ID
            if int(listval[0]) == subject:
                if not start_flag == False:
                    self.start_frame = int(listval[5])
                    start_flag = True
                # print("here")
                cur_pos = ((float(listval[3])+float(listval[1]))/2, (float(listval[4]) + float(listval[2]))/2)
                self.position_list.append(cur_pos)  # update the position_list
                future_pos = ((float(listval_future[3])+float(listval_future[1]))/2,
                            (float(listval_future[4]) + float(listval_future[2]))/2)
                cur_vel = (future_pos[0] - cur_pos[0], future_pos[1] - cur_pos[1])
                cur_speed = np.hypot(cur_vel[0], cur_vel[1])
                speed += cur_speed
                counter += 1
                self.velocity_list.append(cur_vel)

            if int(listval_future[0]) != subject and start_flag==True:
                self.velocity_list[-1] = (0,0)
                self.stop_frame = int(listval[5])
                self.preferred_speed = speed/counter
                self.chosen_destination = cur_pos

                break

    def overlay_video_bbox(self, video_path):
        """
        should plot the subject in the vidoe, ideally just show that part of the video
        :param video_path: path to data file
        :return: None
        """
        if not os.path.isfile(video_path):
            print ("The video file does not exist.")
            return 0
        videocap = cv2.VideoCapture(video_path)
        success, image = videocap.read()
        framecounter = 0
        counter = 0
        # success = True
        while success:
            if self.start_frame <= framecounter <= self.stop_frame:
                cv2.rectangle(image , (int(self.position_list[counter][0])-20, int(self.position_list[counter][1])-20 ),\
                              (int(self.position_list[counter][0])+20 , int(self.position_list[counter][1])+ 20 ),\
                              (0, 255, 0), 2)
                cv2.imshow("frame", image)
                cv2.waitKey(1)
                counter += 1
            framecounter += 1
            success,image = videocap.read()
            if framecounter % 500 == 0:
                print(framecounter)
            if framecounter > self.stop_frame:

                videocap.release()
                cv2.destroyAllWindows()

        return None


def place_annotation(image, annotation_dict, frame, color_dict):
    """
    marks bboxes around the pedestrian on a video sequence
    """
    print(frame)
    font = cv2.FONT_HERSHEY_COMPLEX_SMALL
    frame_info = annotation_dict[frame]
    # print("The frame info :")
    # print(frame_info)
    for subject_set in frame_info:
        if not subject_set[6] and not subject_set[7] and not subject_set[8]:
            cv2.rectangle(image, (int(subject_set[1]), int(subject_set[2])),
                          (int(subject_set[3]), int(subject_set[4])), color_dict[subject_set[9]], 2)
            cv2.putText(image, subject_set[0],
                        (int(subject_set[1])-2, int(subject_set[2])-2), font, 1, color_dict[subject_set[9]])


def plot_annotated_video(videofile, annotation_file):
    color_dict = {'"Biker"': (0,0,255), '"Pedestrian"': (0,255,0), '"Car"': (255,255,0), '"Bus"': (0,0,0),\
                  '"Skateboarder"': (100,100,255), '"Cart"': (255,255,255)}
    if not os.path.isfile(videofile):
        print ("The video file does not exist.")
        return 0
    if not os.path.isfile(annotation_file):
        print ("The annotation file does not exist.")
        return 0

    videocap = cv2.VideoCapture(videofile)
    annotation_list = []
    annotation_dict = {}
    print("Loading annotation file")
    with open(annotation_file) as f:
        for line in f:
            line = line.strip().split(' ')
            annotation_list.append(line)

    # what is this?
    for entry in annotation_list:
        # if 
        frame_id = entry[5]
        if frame_id not in annotation_dict:
            annotation_dict[frame_id] = []
        annotation_dict[frame_id].append(entry)

    annotation_list.sort(key=lambda x: x[5])
    success, image = videocap.read()
    count = 0
    # success = True
    while success:
        place_annotation(image, annotation_dict, str(count), color_dict)
        cv2.imshow("frame", image)
        success, image = videocap.read()
        # cv2.waitKey(1)

        if cv2.waitKey(5) == 27:
            break

        count += 1

    videocap.release()
    cv2.destroyAllWindows()

'''
Each line in an annotations.txt file corresponds to an annotation & contains 10+ columns, separated by spaces. 
The definitions of these columns are:

    1   Track ID. All rows with the same ID belong to the same path.
    2   xmin. The top left x-coordinate of the bounding box.
    3   ymin. The top left y-coordinate of the bounding box.
    4   xmax. The bottom right x-coordinate of the bounding box.
    5   ymax. The bottom right y-coordinate of the bounding box.
    6   frame. The frame that this annotation represents.
    7   lost. If 1, the annotation is outside of the view screen.
    8   occluded. If 1, the annotation is occluded.
    9   generated. If 1, the annotation was automatically interpolated.
    10  label. The label for this annotation, enclosed in quotation marks.
'''

if __name__ == "__main__":

    videofile1 = '../dataset/videos/bookstore/video0/video.mov'
    # videofile = '/media/abhisek/Elements/Local Disk(E) Backup/xx/HoLLywood/Up (2009) [1080p]/up.mp4'
    annotationfile = '../dataset/annotations/bookstore/video0/annotations.txt'

    print("Start")
    plot_annotated_video(videofile1, annotationfile)

    pt1 = Pedestrian()

    pt1.extract_subject_trajectory(annotationfile, 1)

    print(pt1.start_frame)
    print(pt1.stop_frame)
    print(pt1.preferred_speed)
    #print(pt1.position_list)
    #print(pt1.velocity_list)
    # pt1.plotSubjectInvideo(videofile1)


