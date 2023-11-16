import constants, config, ntcore, math
from wpimath.geometry import Pose2d, Rotation2d, Transform2d


class Poses():
    temp_poses = constants.Poses
    grid_pos = None
    station_pos = None
    game_piece_pos = None
    is_red = None
    
    class Type:
        KNodes = 0
        KStation = 1
        KAutoPieces = 2
        
    
    #left to right
    Nodes: dict[int, Pose2d] = {
        1: None,
        2: None,
        3: None,
        4: None,
        5: None,
        6: None,
        7: None,
        8: None,
        9: None,
    }
    
    #left to right, so 1 is single station 2 and 3 are double stations
    Station: dict[int, Pose2d] = {
        1: None,
        2: None,
        3: None,
    }
    
    # left to right
    Auto_Pieces: dict[int, Pose2d] = {
        1: None,
        2: None,
        3: None,
        4: None,
    }
    
    def init(self):
        self.grid_pos, self.station_pos, self.game_piece_pos = self.get_poses()
        
        for i in range(1,10):
            self.Nodes[i] = self.grid_pos[i-1]
            
        for i in range(1,4):
            self.Station[i] = self.station_pos[i-1]
            
        for i in range(1,5):
            self.Auto_Pieces[i] = self.game_piece_pos[i-1]
    
    def get_poses(self):
        poses = self.temp_poses
        april_tags = []
        team = ''
        # Find team and april tags
        if config.active_team == config.Team.blue:
            self.is_red = False
            april_tags = constants.ApriltagPositionDictBlue
            grid_tags = [april_tags[6], april_tags[7], april_tags[8]]
            # grid_tags = [april_tags[6]]
            station_tag = april_tags[4]
            team = 'blue'
            team_station = Pose2d(poses.load_single['blue'], Rotation2d(math.radians(-90)))
        else:
            self.is_red = True
            april_tags = constants.ApriltagPositionDictRed
            grid_tags = [april_tags[3], april_tags[2], april_tags[1]]
            station_tag = april_tags[5]
            team = 'red'
            team_station = Pose2d(poses.load_single['red'], Rotation2d(math.radians(90)))
            
        # find grid targets
        grid_pos = []
        for tag in grid_tags:
            tag2d = tag.toPose2d()
            if team == 'red':
                grid_pos.append(Pose2d(tag2d.X() + poses.node_left.X(), tag2d.Y() + poses.node_left.Y(), 0 ))
                grid_pos.append(Pose2d(tag2d.X() + poses.node_front.X(), tag2d.Y(), 0))
                grid_pos.append(Pose2d(tag2d.X() + poses.node_right.X(), tag2d.Y() + poses.node_right.Y(), 0))
            else:
                grid_pos.append(Pose2d(tag2d.X() + poses.node_right.X(), tag2d.Y() + poses.node_right.Y(), 0))
                grid_pos.append(Pose2d(tag2d.X() + poses.node_front.X(), tag2d.Y(), 0))
                grid_pos.append(Pose2d(tag2d.X() + poses.node_left.X(), tag2d.Y() + poses.node_left.Y(), 0 ))
                

        # find station targets (single station, double stations)
        station_pos = []
        station_pos.append(team_station.relativeTo(station_tag.toPose2d()))
        station_pos.append(Pose2d(poses.load_double_left, Rotation2d(math.radians(0))).relativeTo(station_tag.toPose2d()))
        station_pos.append(Pose2d(poses.load_double_right, Rotation2d(math.radians(0))).relativeTo(station_tag.toPose2d()))
        
        game_piece_pos = []
        
        if team == 'blue':
            game_piece_pos = [
                Pose2d(poses.far_left_piece_auto[team], Rotation2d()),
                Pose2d(poses.center_left_piece_auto[team], Rotation2d()),
                Pose2d(poses.center_right_piece_auto[team], Rotation2d()),
                Pose2d(poses.far_right_piece_auto[team], Rotation2d())
            ]
        else:
            game_piece_pos = [
                Pose2d(poses.far_right_piece_auto[team], Rotation2d()),
                Pose2d(poses.center_right_piece_auto[team], Rotation2d()),
                Pose2d(poses.center_left_piece_auto[team], Rotation2d()),
                Pose2d(poses.far_left_piece_auto[team], Rotation2d()),
                
            ]
        

        tot_pos: list[Pose2d] = station_pos + grid_pos + game_piece_pos

        send_pose = []
        for i in tot_pos:
            send_pose.append(i.X()),
            send_pose.append(i.Y()),
            send_pose.append(i.rotation().radians())

        ntcore.NetworkTableInstance.getDefault().getTable('auto').putNumberArray(
            "POI", send_pose 
        )

        send_tags = []
        for tag in april_tags.values():
            tag = tag.toPose2d()
            send_tags.append(tag.X())
            send_tags.append(tag.Y())
            send_tags.append(tag.rotation().radians())
            
        ntcore.NetworkTableInstance.getDefault().getTable('auto').putNumberArray(
            'april_tags', send_tags
        )

        return grid_pos, station_pos, game_piece_pos
    
    def poses_created(self):
        if self.grid_pos == None or self.station_pos == None or self.game_piece_pos == None:
            return False
        else:
            return True
        
    def get_selected_POI(self, pose: tuple[Type, int]):
            pose_type, num = pose
            if pose_type == Poses.Type.KNodes:
                return self.Nodes[num]
            elif pose_type == Poses.Type.KStation:
                return self.Station[num]
            elif pose_type == Poses.Type.KAutoPieces:
                return self.Auto_Pieces[num]
    
    
    def get_grid(self, node):
        return self.Nodes[node]
    
    def get_station(self, station):
        return self.Station[station]
    
    def get_game_piece(self, piece):
        return self.Auto_Pieces[piece]
    
    def return_poses(self):
        if self.grid_pos is None or self.station_pos is None or self.game_piece_pos is None:
            self.grid_pos, self.station_pos, self.game_piece_pos = self.get_poses()
        return self.grid_pos, self.station_pos, self.game_piece_pos
