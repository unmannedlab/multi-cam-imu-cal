# Software Architecture  {#software_architecture}


## Sensor Initialization

@startuml

participant xCalibrateNode as XCN
participant CalibrationEKF as EKF

XCN -> EKF : Register Sensor
EKF -> EKF : Initialize New Sensor
XCN <- EKF : Sensor index

@enduml

## Update Sequence

@startuml

participant sensor 
participant xCalibrateNode as XCN
participant CalibrationEKF as EKF

sensor -> sensor : Measurement
sensor -> XCN : Sensor Callback
XCN -> EKF : Sensor Callback
EKF -> EKF : Propagate Time
EKF -> EKF : Update State
XCN <-- EKF 

@enduml 


## Sensor class diagram with associations

@startuml

object RigidBody
object DynamicSensor
object IntrinsicSensor
object ExtrinsicSensor
object BaseIMU
object IMU 
object Camera 
object Lidar 

DynamicSensor <|-- ExtrinsicSensor
DynamicSensor <|-- IntrinsicSensor
IntrinsicSensor <|-- BaseIMU

IntrinsicSensor <|-- IMU
ExtrinsicSensor <|-- Camera
ExtrinsicSensor <|-- Lidar

RigidBody <-- ExtrinsicSensor
RigidBody <-- IntrinsicSensor

@enduml