import ntcore
from robotpy_toolkit_7407.motors import SparkMaxConfig
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile

class PIDConfigEditor:
    
    nt = ntcore.NetworkTableInstance.getDefault()
    table = nt.getTable("PID")
    
    def editSparkMaxConfig(self, config: SparkMaxConfig, name: str):
        '''
        Wrapper for SparkMaxConfig that edits a SparkMaxConfig object using the SmartDashboard
        Edits a SparkMaxConfig object using the SmartDashboard
        This should be called in robotPeriodic(), and does not need to be returned to a variable, since python is pass-by-reference
        @param config: The SparkMaxConfig object to edit
        @param name: The name of the SparkMaxConfig object
        @return: The edited SparkMaxConfig object
        '''
        table = self.table.getSubTable(name)
        
        table.setDefaultNumber("kP", config.kP)
        table.setDefaultNumber("kI", config.kI)
        table.setDefaultNumber("kF", config.kF)
        table.setDefaultNumber("kD", config.kD)
        table.setDefaultNumber("min_output", config.output_range[0])
        table.setDefaultNumber("max_output", config.output_range[1])
        
        config.kP = table.getNumber("kP", config.kP)
        config.kI = table.getNumber("kI", config.kI)
        config.kD = table.getNumber("kD", config.kD)
        config.kF = table.getNumber("kF", config.kF)
        config.output_range = (table.getNumber("min_output", config.output_range[0]), table.getNumber("max_output", config.output_range[1]))
        
        return config
    
    def editProfiledPIDController(self, controller: ProfiledPIDController, max_v: int, max_a:int, name: str):
        '''
        Wrapper for ProfiledPIDController that edits a ProfiledPIDController object using the SmartDashboard
        This should be called in robotPeriodic(), and does not need to be returned to a variable, since python is pass-by-reference
        @param controller: The ProfiledPIDController object to edit
        @param max_v: The maximum velocity of the ProfiledPIDController
        @param max_a: The maximum acceleration of the ProfiledPIDController
        @param name: The name of the ProfiledPIDController object
        @return: The edited ProfiledPIDController object'''
        
        table = self.table.getSubTable(name)
        
        table.setDefaultNumber("kP", controller.getP())
        table.setDefaultNumber("kI", controller.getI())
        table.setDefaultNumber("kD", controller.getD())
        table.setDefaultNumber("max_v", max_v)
        table.setDefaultNumber("max_a", max_a)
        table.setDefaultNumber("V_tolerance", controller.getVelocityTolerance())
        table.setDefaultNumber("P_tolerance", controller.getPositionTolerance())
        
        controller.setP(table.getNumber("kP", controller.getP()))
        controller.setI(table.getNumber("kI", controller.getI()))
        controller.setD(table.getNumber("kD", controller.getD()))
        controller.setConstraints(TrapezoidProfile.Constraints(table.getNumber("max_v", max_v), table.getNumber("max_a", max_a)))
        controller.setTolerance(table.getNumber("V_tolerance", controller.getVelocityTolerance()), table.getNumber("P_tolerance", controller.getPositionTolerance()))
        
        return controller
        
        
        
        
    
