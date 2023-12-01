use serialport::available_ports;

pub fn get_usb_serial_devices() -> Vec<serialport::SerialPortInfo> {
    match available_ports() {
        Ok(ports) => {
            let usb_ports: Vec<_> = ports
                .into_iter()
                .filter(|port| matches!(port.port_type, serialport::SerialPortType::UsbPort(_)))
                .collect();

            usb_ports
        }
        Err(err) => {
            eprintln!("Error listing serial devices: ({:?})", err);
            Vec::new()
        }
    }
}

pub fn print_serial_devices() {
    let usb_ports = get_usb_serial_devices();
    if usb_ports.is_empty() {
        println!("No known USB serial devices found.");
        return;
    }

    println!("Known USB serial devices:");
    for port in &usb_ports {
        println!("{}", port.port_name);
    }
}
