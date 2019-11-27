'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import leftBackToStand
from os import listdir
import pickle


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = 'robot_pose.pkl' # LOAD YOUR CLASSIFIER
        

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        dataset = []
        ROBOT_POSE_DATA_DIR = 'robot_pose_data'
        classes = listdir(ROBOT_POSE_DATA_DIR)
        features = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch']
        clf = pickle.load(open(self.posture_classifier))
        

        for i in features:
            dataset.append(perception.joint[i])

            
        #add x and y
        dataset.append(perception.imu[0])
        dataset.append(perception.imu[1])

        
        all_dataset = []
        all_dataset.append(dataset)
        predicted = clf.predict(all_dataset)
        posture = classes[predicted[0]]

        return posture


if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
