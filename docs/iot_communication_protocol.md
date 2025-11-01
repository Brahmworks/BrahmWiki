# Device Communication JSON Architecture

Based on industry standards and best practices for IoT device communication, this document outlines a comprehensive JSON architecture that incorporates standardized approaches for device handshakes, health checks, and command handling.

## **Device Handshake Protocol**

The handshake establishes initial communication and authentication between the device and the server.

```json
{
  "messageType": "handshake",
  "version": "1.0",
  "request": {
    "deviceId": "device_12345",
    "deviceType": "sensor",
    "firmwareVersion": "2.1.4"
  },
  "response": {
    "status": "accepted",
    "statusCode": 200,
    "sessionId": "sess_67890",
    "serverTime": "2025-11-01T16:40:00.500Z",
    "heartbeatInterval": 30,
    "commandEndpoint": "mqtt://broker.example.com:8883/device/12345/commands",
    "healthEndpoint": "https://api.example.com/device/12345/health",
    "configuration": {
      "reportingInterval": 60
    }
  }
}
```

## **Device Health Check Protocol**

Following RFC health check standards for consistent monitoring.

**Request:**
```json
{
  "messageType": "health_check",
  "version": "1.0",
  "deviceId": "device_12345"
}
```

**Response:**
```json
{
  "messageType": "health_check",
  "version": "1.0",
  "deviceId": "device_12345",
  "status": "pass",
  "statusCode": 200,
  "checks": {
    "sensor:sensor_1": [
      {
        "componentId": "sensor_1",
        "componentType": "sensor",
        "status": "pass",
        "observedValue": 75.0
      }
    ],
    "sensor:sensor_2": [
      {
        "componentId": "sensor_2",
        "componentType": "sensor",
        "status": "pass",
        "observedValue": 25.0
      }
    ],
    "sensor:sensor_3": [
      {
        "componentId": "sensor_3",
        "componentType": "sensor",
        "status": "pass",
        "observedValue": 15.2
      }
    ],
    "actuator:actuator_1": [
      {
        "componentId": "actuator_1",
        "componentType": "actuator",
        "status": "pass",
        "observedValue": "off"
      }
    ],
    "actuator:actuator_2": [
      {
        "componentId": "actuator_2",
        "componentType": "actuator",
        "status": "pass",
        "observedValue": "closed"
      }
    ],
    "actuator:actuator_3": [
      {
        "componentId": "actuator_3",
        "componentType": "actuator",
        "status": "pass",
        "observedValue": "off"
      }
    ]
  }
}
```

## **Command Message Formats**

Based on SAP IoT and industry standards for device command structures.

### **Control Command**
```json
{
  "messageType": "command",
  "commandType": "control",
  "version": "1.0",
  "commandId": "cmd_789123",
  "deviceId": "device_12345",
  "command": {
    "action": "set_state",
    "parameters": {
      "component": "actuator_1",
      "state": "on"
    }
  },
  "response": {
    "statusCode": 200,
    "status": "success"
  }
}
```

### **Configuration Command**
```json
{
  "messageType": "command",
  "commandType": "configuration",
  "version": "1.0",
  "commandId": "cmd_789124",
  "deviceId": "device_12345",
  "command": {
    "action": "update_config",
    "parameters": {
      "reportingInterval": 120
    }
  },
  "response": {
    "statusCode": 200,
    "status": "success"
  }
}
```

### **Firmware Update Command**
```json
{
  "messageType": "command",
  "commandType": "firmware_update",
  "version": "1.0",
  "commandId": "cmd_789125",
  "deviceId": "device_12345",
  "command": {
    "action": "update_firmware",
    "parameters": {
      "firmwareVersion": "2.2.0",
      "downloadUrl": "https://firmware.example.com/device/2.2.0.bin"
    }
  },
  "response": {
    "statusCode": 202,
    "status": "accepted"
  }
}
```

### **Data Request Command**
```json
{
  "messageType": "command",
  "commandType": "data_request",
  "version": "1.0",
  "commandId": "cmd_789126",
  "deviceId": "device_12345",
  "command": {
    "action": "get_sensor_data",
    "parameters": {
      "sensors": ["sensor_1", "sensor_2"]
    }
  },
  "response": {
    "statusCode": 200,
    "status": "success",
    "data": {
      "sensor_1": [
        {"timestamp": "2025-11-01T16:00:00.000Z", "value": 22.3},
        {"timestamp": "2025-11-01T16:30:00.000Z", "value": 23.1}
      ],
      "sensor_2": [
        {"timestamp": "2025-11-01T16:00:00.000Z", "value": 65},
        {"timestamp": "2025-11-01T16:30:00.000Z", "value": 67}
      ]
    }
  }
}
```

## **Standard Status Codes**

Following HTTP and REST API standards for consistent status reporting.

### **Success Codes (2xx)**
- **200 OK**: Command executed successfully
- **201 Created**: Resource created successfully  
- **202 Accepted**: Command accepted, processing asynchronously
- **204 No Content**: Command executed, no response body

### **Client Error Codes (4xx)**
- **400 Bad Request**: Invalid command format or parameters
- **401 Unauthorized**: Authentication required or failed
- **403 Forbidden**: Command not permitted for this device
- **404 Not Found**: Device or resource not found
- **408 Request Timeout**: Command execution timeout
- **429 Too Many Requests**: Rate limiting applied

### **Server Error Codes (5xx)**
- **500 Internal Server Error**: Server processing error
- **502 Bad Gateway**: Communication error with device
- **503 Service Unavailable**: Service temporarily unavailable
- **504 Gateway Timeout**: Device communication timeout

### **Device-Specific Status Codes**
- **600 Device Offline**: Device not reachable
- **601 Low Battery**: Command rejected due to low power
- **602 Sensor Error**: Hardware sensor malfunction
- **603 Memory Full**: Insufficient storage for command
- **604 Invalid State**: Device not in appropriate state

## **Error Response Format**

```json
{
  "messageType": "error",
  "version": "1.0",
  "timestamp": "2025-11-01T16:40:00.000Z",
  "statusCode": 400,
  "error": {
    "code": "INVALID_COMMAND_FORMAT",
    "message": "Required field 'deviceId' is missing",
    "details": {
      "field": "deviceId",
      "expectedType": "string",
      "receivedValue": null
    },
    "retryable": false,
    "documentation": "https://api.example.com/docs/errors/400"
  },
  "requestId": "req_456789",
  "deviceId": "device_12345"
}
```

This architecture provides a comprehensive, standards-based approach to IoT device communication using JSON. It incorporates the IETF health check format, follows REST API status code conventions, and uses proven patterns from industrial IoT implementations. The format is lightweight, human-readable, and supports the 75% adoption rate of JSON in connected devices, making it ideal for resource-constrained environments while maintaining interoperability across different platforms and tooling systems.
