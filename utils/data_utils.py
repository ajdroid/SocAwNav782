import numpy as np 
import cv2
import os 
import sys
import cost_map
from matplotlib import pyplot as plt
from matplotlib import cm
import search
import time


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
                annotation_list.append(line)

        # The position is the midpoint of the bounding box
        for i in range(len(annotation_list)-1):  # Skips last frame
            # old_pos = (-1, -1)
            listval = annotation_list[i]
            listval_future = annotation_list[i+1]
            speed = 0
            counter = 0
            # get all annotations for this subject ID
            # import ipdb; ipdb.set_trace();
            if int(listval[0]) == subject:
                if start_flag == False:
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
            else:
                continue

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
    plot the pedestrian past trajectories
    """
    print(frame)
    font = cv2.FONT_HERSHEY_COMPLEX_SMALL
    frame_info = annotation_dict[frame]
    # print("The frame info :")
    # print(frame_info)

    for subject_set in frame_info:
        # print(subject_set[9], subject_set[9] == '"Pedestrian"')
        if subject_set[6] != 1 and subject_set[7] != 1 and subject_set[8] != 1 and subject_set[9] == '"Pedestrian"':
            cv2.rectangle(image, (int(subject_set[1]), int(subject_set[2])),
                          (int(subject_set[3]), int(subject_set[4])), color_dict[subject_set[9]]\
                          , thickness=3)
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
        cv2.waitKey(1)

        if cv2.waitKey(5) == 27:
            break

        count += 1

    videocap.release()
    cv2.destroyAllWindows()


def plot_annotated_video_plt(videofile, annotation_file):
    # this is BGR
    color_dict = {'"Biker"': (0, 0, 0), '"Pedestrian"': (0, 0, 255), '"Car"': (255, 255, 0), '"Bus"': (0, 0, 0), \
                  '"Skateboarder"': (100, 100, 255), '"Cart"': (255, 255, 255)}
    if not os.path.isfile(videofile):
        print("The video file does not exist.")
        return 0
    if not os.path.isfile(annotation_file):
        print("The annotation file does not exist.")
        return 0

    obstacle_map = cv2.imread("../obstacleMask.jpg", 0)
    obstacle_map = obstacle_map/obstacle_map.max()
    cost_scale = 100000
    # goalX = 1204
    # goalY = 300
    # goalX = 100
    # goalY = 1000
    # startX = 1356
    # startY = 629
    startX = 400
    startY = 120
    goalX = 1000
    goalY = 1000

    start_frame = 9060

    if obstacle_map[startY, startX] or obstacle_map[goalY, goalX]:
        print(" Start or goal is occluded!!")
        exit(0)
    curX, curY = startX, startY

    # load in the video
    videocap = cv2.VideoCapture(videofile)
    vid_length = int(videocap.get(cv2.CAP_PROP_FRAME_COUNT))

    annotation_list = []
    annotation_dict = {}
    pedestrian_dictx = {}
    pedestrian_dicty = {}
    print("Loading annotation file")
    with open(annotation_file) as f:
        for line in f:
            line = line.strip().split(' ')
            annotation_list.append(line)

    # initialize loading the annotations
    for entry in annotation_list:
        frame_id = entry[5]
        if frame_id not in annotation_dict:
            annotation_dict[frame_id] = []
        track_id = entry[0]
        if track_id not in pedestrian_dictx:
            pedestrian_dictx[track_id] = np.empty((vid_length,))
            pedestrian_dictx[track_id].fill(np.nan)
            pedestrian_dicty[track_id] = np.empty((vid_length,))
            pedestrian_dicty[track_id].fill(np.nan)

        pedestrian_dictx[track_id][int(frame_id)] = (int(entry[1]) +int(entry[3])) / 2
        pedestrian_dicty[track_id][int(frame_id)] = (int(entry[2]) + int(entry[4])) / 2

        annotation_dict[frame_id].append(entry)
    annotation_list.sort(key=lambda x: x[5])

    success, image = videocap.read()
    height, width = image.shape[:2]

    # initialize loading the cost maps
    cm_object = cost_map.CostMap(height, width, 100)
    X = np.linspace(0, width, width)
    Y = np.linspace(0, height, height)
    X, Y = np.meshgrid(X, Y)

    count = 0
    plt.figure()
    plan = np.zeros((2, 1))
    local_plan = np.zeros((2, 10))

    videocap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
    count = start_frame
    timers= []

    while success:

        # cost maps available only from 80
        if count < start_frame:
            success, image = videocap.read()
            count += 1
            continue
        # print("read 80")
        place_annotation(image, annotation_dict, str(count), color_dict)

        # cv2.imshow("frame", image)

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # invert channels to use with pyplot
        if count %10 == 0:
            plt.gcf().clear()
            pdf_plot = cm_object.get_cost_map(count)*cost_scale
            pdf = pdf_plot + obstacle_map[:, :, None]*1e5
            pdf = np.sum(pdf, axis=2, keepdims=True)#[:,:,None]
            print(pdf.shape)
            pdf = np.tile(pdf, (1,1,12))
            print(pdf.shape)
            input_costmap = np.swapaxes(pdf, 0, 1)

            # shift the odf time axis to be consistent with planner: move from back to front
            # input_costmap = input_costmap*1000000
            pred_times = np.arange(0, 10)
            speed = 5

            start = time.time()
            local_plan = search.graphSearch(speed, curX, curY, goalX, goalY, input_costmap, pred_times)
            end = time.time()
            print(end-start)
            timers.append(end-start)
            plan = np.append(plan, local_plan[:2, :], axis=1)


            print(local_plan)
            if local_plan.shape[1] == 1:
                if local_plan[0, 0] == goalX and local_plan[1, 0] == goalY:
                    print("Reached Goal!")
                    print("Average planning times: {}".format(np.mean(timers)))
                    plt.contourf(X, Y, np.sum(pdf, axis=2), zdir='z', cmap=cm.viridis, alpha=0.4)
                    plt.colorbar()
                    plt.imshow(image, alpha=0.6)
                    plt.plot(*(plan[:, 1:]), linewidth=5, color='g')
                    # plot goal location
                    plt.scatter(*(local_plan[:2, :]), s=10)

                    # plot the pedestrian trajectories
                    for anno in annotation_dict[str(count)]:
                        ped_id = anno[0]
                        plt.plot(pedestrian_dictx[ped_id][start_frame+1:count],\
                                 pedestrian_dicty[ped_id][start_frame+1:count],\
                                 linewidth=2, color='r', alpha=0.1)

                    plt.show()

                    break

            curX = int(local_plan[0, -1])
            curY = int(local_plan[1, -1])

        # plotting paths to now
            # pdf = cm_object.get_cost_map(count)
            plt.contourf(X, Y, np.sum(pdf_plot, axis=2), zdir='z', cmap=cm.viridis, alpha=0.4)
            plt.colorbar()
            plt.imshow(image, alpha=0.6)
            plt.plot(*(plan[:, 1:]), linewidth=5, color='y')
            plt.scatter(goalX, goalY, marker='*', c='w', s=200)

            plt.pause(0.5)


        success, image = videocap.read()
        count += 1

        if cv2.waitKey(5) == 27:
            break

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
    annotationfile = '../dataset/annotations/bookstore/video0/annotations.txt'

    print("Start")
    # plot_annotated_video(videofile1, annotationfile)
    plot_annotated_video_plt(videofile1, annotationfile)

    # pt1 = Pedestrian()

    # pt1.extract_subject_trajectory(annotationfile, 1)
    #
    # print(pt1.start_frame)
    # print(pt1.stop_frame)
    # print(pt1.preferred_speed)
    # print(pt1.position_list)
    # print(pt1.velocity_list)
    # pt1.plotSubjectInvideo(videofile1)



