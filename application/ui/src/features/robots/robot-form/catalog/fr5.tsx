import { TextField } from '@geti-ui/ui';

import type { SchemaRobot, SchemaRobotInput, SchemaRobotType } from '../../robot-types';
import { useRobotFormFields } from '../provider';

/** Controller IP the Fairino SDK talks XML-RPC to; also the WebApp address. */
const DEFAULT_FR5_IP = '192.168.58.2';

export interface FR5FormData {
    name: string;
    connection_string: string;
    serial_number: string;
}

export const getInitialFR5FormData = (robot?: SchemaRobot): FR5FormData => ({
    name: robot?.name ?? '',
    connection_string: robot && 'connection_string' in robot.payload ? robot.payload.connection_string : DEFAULT_FR5_IP,
    serial_number: robot?.payload?.serial_number ?? '',
});

export const buildFR5Body = (
    formData: FR5FormData,
    schemaType: SchemaRobotType,
    robot_id: string
): SchemaRobotInput | null => {
    if (!formData.connection_string) {
        return null;
    }

    return {
        id: robot_id,
        name: formData.name,
        type: schemaType,
        payload: {
            connection_string: formData.connection_string,
            // IP-addressed robot; the backend keeps the field for schema symmetry.
            serial_number: formData.serial_number ?? '',
        },
    } as SchemaRobotInput;
};

export const FR5FormFields = () => {
    const { formData, updateField } = useRobotFormFields<FR5FormData>();

    // No Identify button: /api/hardware/identify only dispatches for SO101 and
    // Trossen, so it would silently no-op here rather than move the arm.
    return (
        <TextField
            isRequired
            label='Robot IP address'
            width='100%'
            value={formData.connection_string}
            onChange={(connection_string) => {
                updateField('connection_string', connection_string);
            }}
            placeholder={DEFAULT_FR5_IP}
        />
    );
};
