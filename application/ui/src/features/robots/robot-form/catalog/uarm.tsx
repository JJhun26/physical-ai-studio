import { Flex, Item, Picker, Text } from '@geti-ui/ui';

import { $api } from '../../../../api/client';
import type { SchemaRobot, SchemaRobotInput, SchemaRobotType } from '../../robot-types';
import { useRobotFormFields } from '../provider';
import { RefreshRobotsButton } from './actions';

export interface UArmFormData {
    name: string;
    serial_number: string;
    connection_string: string;
}

export const getInitialUArmFormData = (robot?: SchemaRobot): UArmFormData => ({
    name: robot?.name ?? '',
    serial_number: robot?.payload?.serial_number ?? '',
    connection_string: robot && 'connection_string' in robot.payload ? robot.payload.connection_string : '',
});

export const buildUArmBody = (
    formData: UArmFormData,
    schemaType: SchemaRobotType,
    robot_id: string
): SchemaRobotInput | null => {
    // The backend resolves the leader by serial port, so a port is mandatory even
    // though the payload also accepts a serial number.
    if (!formData.connection_string) {
        return null;
    }

    return {
        id: robot_id,
        name: formData.name,
        type: schemaType,
        payload: {
            connection_string: formData.connection_string,
            serial_number: formData.serial_number ?? '',
        },
    } as SchemaRobotInput;
};

const getDeviceKey = ({
    serial_number,
    connection_string,
}: {
    serial_number: string;
    connection_string: string | null;
}) => {
    if (serial_number !== '') {
        return `serial:${serial_number}`;
    }
    return `port:${connection_string ?? ''}`;
};

export const UArmFormFields = () => {
    const serialDevicesQuery = $api.useSuspenseQuery('get', '/api/hardware/serial_devices');

    const { formData, updateField } = useRobotFormFields<UArmFormData>();

    const selectedKey =
        formData.serial_number !== '' || formData.connection_string !== ''
            ? getDeviceKey({
                  serial_number: formData.serial_number,
                  connection_string: formData.connection_string,
              })
            : null;

    // No Identify button: the leader is back-driven with torque disabled, so
    // "move a joint to identify it" does not apply.
    return (
        <Flex gap='size-100' justifyContent={'space-between'} alignItems={'end'}>
            <Picker
                name='payload.device_key'
                label='Select leader arm'
                isRequired
                width='100%'
                selectedKey={selectedKey}
                onSelectionChange={(key) => {
                    const device = serialDevicesQuery.data.find(
                        (d) =>
                            getDeviceKey({
                                serial_number: d.serial_number ?? '',
                                connection_string: d.connection_string,
                            }) === String(key)
                    );

                    if (device === undefined) {
                        return;
                    }

                    updateField('serial_number', device.serial_number ?? '');
                    updateField('connection_string', device.connection_string ?? '');
                }}
            >
                {serialDevicesQuery.data.map((serial_device) => {
                    const serial_number = serial_device.serial_number ?? '';
                    const label = serial_number !== '' ? serial_number : 'No serial number';

                    return (
                        <Item
                            key={getDeviceKey({
                                serial_number,
                                connection_string: serial_device.connection_string,
                            })}
                            textValue={label}
                        >
                            <Text>{label}</Text>
                            <Text slot='description'>{serial_device.connection_string ?? ''}</Text>
                        </Item>
                    );
                })}
            </Picker>

            <Flex gap='size-100'>
                <RefreshRobotsButton />
            </Flex>
        </Flex>
    );
};
