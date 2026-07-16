import {
    SchemaFr5RobotInput,
    SchemaFr5RobotOutput,
    SchemaFr5RobotWithConnectionState,
    SchemaSo101RobotInput,
    SchemaSo101RobotOutput,
    SchemaSo101RobotWithConnectionState,
    SchemaTrossenBimanualRobotInput,
    SchemaTrossenBimanualRobotOutput,
    SchemaTrossenBimanualRobotWithConnectionState,
    SchemaTrossenSingleArmRobotInput,
    SchemaTrossenSingleArmRobotOutput,
    SchemaTrossenSingleArmRobotWithConnectionState,
    SchemaUArmRobotInput,
    SchemaUArmRobotOutput,
    SchemaUArmRobotWithConnectionState,
} from '../../api/openapi-spec';

/** Union of all concrete robot output schemas (as returned by the API). */
export type SchemaRobot =
    | SchemaSo101RobotOutput
    | SchemaTrossenSingleArmRobotOutput
    | SchemaTrossenBimanualRobotOutput
    | SchemaFr5RobotOutput
    | SchemaUArmRobotOutput;

/** Union of all concrete robot input schemas (for create/update requests). */
export type SchemaRobotInput =
    | SchemaSo101RobotInput
    | SchemaTrossenSingleArmRobotInput
    | SchemaTrossenBimanualRobotInput
    | SchemaFr5RobotInput
    | SchemaUArmRobotInput;

/** All possible robot type discriminators. */
export type SchemaRobotType = SchemaRobot['type'];

/** Union of all robot-with-connection-state schemas (as returned by the online endpoint). */
export type SchemaRobotWithConnectionState =
    | SchemaSo101RobotWithConnectionState
    | SchemaTrossenSingleArmRobotWithConnectionState
    | SchemaTrossenBimanualRobotWithConnectionState
    | SchemaFr5RobotWithConnectionState
    | SchemaUArmRobotWithConnectionState;
