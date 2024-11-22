import env, sensors, features
import random
import pygame

def random_color():
    levels = range(32, 256, 32)
    return tuple(random.choice(levels) for _ in range(3))

FeatureMAP = features.featureDetection()
environment = env.buildEnvironment((600, 1200))
originalMap = environment.map.copy()
laser = sensors.LaserSensor(200, originalMap, uncertainty=(0.5, 0.01))
environment.map.fill((255,255,255))
environment.infomap = environment.map.copy()
originalMap = environment.map.copy()

running = True
FEATURE_DECTECTION = True
BREAK_POINT_IND = 0

while running:
    environment.infomap=originalMap.copy()
    FEATURE_DECTECTION = True
    BREAK_POINT_IND = 0
    ENDPOINTS = [0,0]
    sensorON = False
    PREDICTED_POINTS_TODRAW = []

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    if pygame.mouse.get_focused():
        sensorON = True
    elif not pygame.mouse.get_focused():
        sensorON = False

    if sensorON:
        position = pygame.mouse.get_pos()
        laser.position = position
        sensor_data = laser.sense_obstacles()
        FeatureMAP.laser_points_set(sensor_data)
        # print("MAIN.PY .. sensor data -->", sensor_data)

        while BREAK_POINT_IND < (FeatureMAP.NP - FeatureMAP.PMIN):
            seedSeg = FeatureMAP.seed_segment_detection(laser.position, BREAK_POINT_IND)
            # print("MAIN.PY .. start seg")
            if seedSeg == False:
                break
            else:
                seedSegment = seedSeg[0]
                PREDICTED_POINTS_TODRAW = seedSeg[1]
                INDICES = seedSeg[2]

                results = FeatureMAP.seed_segment_growing(INDICES, BREAK_POINT_IND)
                # print('grow seg')

                if (results == False) or (results == None):
                    # print('no results. breaking from seg')
                    BREAK_POINT_IND = INDICES[1]
                    continue
                else:
                    # print("MAIN.PY .. RESULTS")
                    # print(results)
                    line_eq = results[1]
                    m, c = results[5]
                    line_seg = results[0]
                    OUTERMOST = results[2]
                    BREAK_POINT_IND = results[3]

                    ENDPOINTS[0] = FeatureMAP.projection_point2line(OUTERMOST[0], m, c)
                    ENDPOINTS[1] = FeatureMAP.projection_point2line(OUTERMOST[1], m, c)

                    FeatureMAP.FEATURES.append([[m,c], ENDPOINTS])
                    pygame.draw.line(environment.infomap, (0,255,0), ENDPOINTS[0], ENDPOINTS[1], 1)
                    environment.data_storage(sensor_data)

                    FeatureMAP.FEATURES=FeatureMAP.lineFeats2point()
                    features.landmark_association(FeatureMAP.FEATURES) # might need to make first "Features" -> "FEATURES"

        for landmark in features.Landmarks:
            pygame.draw.line(environment.infomap, (0,0,255), landmark[1][0], landmark[1][1], 2) # may need to make "landmark" -> "landmarks"

    environment.map.blit(environment.infomap, (0,0))
    pygame.display.update()
