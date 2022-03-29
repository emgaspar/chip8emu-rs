use std::fs::File;
use std::io::Read;
use imgui::*;

mod chip8;
mod support;

const WHITE: [f32; 4] = [1.0, 1.0, 1.0, 1.0];
const BLACK: [f32; 4] = [0.0, 0.0, 0.0, 1.0];
const CYAN:  [f32; 4] = [0.0, 1.0, 1.0, 1.0];
const MAGENTA: [f32; 4] = [1.0, 0.502, 0.957, 1.0];
const GREEN: [f32; 4] = [0.0, 1.0, 0.0, 1.0];
const RED: [f32; 4] = [1.0, 0.149, 0.447, 1.0];
const YELLOW: [f32; 4] = [1.0, 1.0, 0.0, 1.0];
const GREY: [f32; 4] = [0.5, 0.5, 0.5, 1.0];


enum EmulationMode {
    Stop,
    Run,
    Step
}

fn main() {
    let mut chip8_emu : chip8::CPU = chip8::CPU::new();
    let mut file = File::open("./roms/test_opcode.ch8").expect("File not found");
    //let mut file = File::open("./roms/test_opcode.ch8").expect("File not found");
    let mut rom = Vec::new();
    file.read_to_end(&mut rom).unwrap();

    chip8_emu.load_rom(&rom);

    let system = support::init("Chip-8 Emulator");
    let mut emulation_mode: EmulationMode = EmulationMode::Stop;
    system.main_loop(move |_, ui| {

        show_control_emulation(&mut chip8_emu, ui, &mut emulation_mode);

        // Run emulator during 10 ticks
        match &emulation_mode {
            EmulationMode::Run => {
                for _ in 0 .. 10 {
                    chip8_emu.run_tick();
                }
            },
            EmulationMode::Step => {
                chip8_emu.run_tick();
                
            }
            _ => {} 
        }

        show_chip8_cpu(&mut chip8_emu, ui);

        show_chip8_output(&chip8_emu, ui);

        show_memory_disassembled(&chip8_emu, ui);

        show_memory(&chip8_emu, ui);
    });
}

fn show_control_emulation(chip8_emu: &mut chip8::CPU, ui: &mut Ui, emulation_mode: &mut EmulationMode) {

    let w = Window::new("Chip-8 Emulation Control")
        .position([600.0, 600.0], Condition::FirstUseEver)
        .scroll_bar(false)
        .resizable(false);
    w.build(ui, || {
        if ui.button("Run") {
            *emulation_mode = EmulationMode::Run; 
            for _ in 0 .. 10 {
               chip8_emu.run_tick();
            }
        }
        ui.same_line();
        if ui.button("Step") { chip8_emu.run_tick(); }
        ui.same_line();
        if ui.button("Stop") { *emulation_mode = EmulationMode::Stop; }
        if ui.button("Reset") { 
            *emulation_mode = EmulationMode::Stop;
            chip8_emu.reset(); 
        }
    });
}

fn show_chip8_output(chip8_emu: & chip8::CPU, ui: &mut Ui) {
    let w = Window::new("Output")
        .size([660.0, 360.0], Condition::FirstUseEver)
        .scroll_bar(false)
        .resizable(false);

    w.build(ui, || {
        let draw_list = ui.get_window_draw_list();

        let canvas_pos = ui.cursor_screen_pos();
        let screen = chip8_emu.get_screen();

        // A frame is drawn around the CHIP-8 screen
        draw_list.add_rect(
            canvas_pos, 
            [canvas_pos[0] + (10.0 * chip8::SCREEN_SIZE.0 as f32), 
                canvas_pos[1] + (10.0 * chip8::SCREEN_SIZE.1 as f32)],
            WHITE).build();

        for y in 0 .. chip8::SCREEN_SIZE.1 {
            for x in 0 .. chip8::SCREEN_SIZE.0 {
                let color = 
                    if screen[y*chip8::SCREEN_SIZE.0 + x] { BLACK } else { WHITE };
                let left_up = 
                    [canvas_pos[0] + (x as f32) * 10.0,
                     canvas_pos[1] + (y as f32) * 10.0];
                let right_down =  
                    [canvas_pos[0] + ((x + 1) as f32) * 10.0,
                     canvas_pos[1] + ((y + 1) as f32) * 10.0];
                draw_list.add_rect_filled_multicolor(
                    left_up, right_down, 
                    color, color, 
                    color, color);
            }
        }
    });
}

//TODO: To macro
fn u16_to_binary_spaced(value: u16) -> String {
    format!("{:04b} {:04b} {:04b} {:04b}",
            value >> 12, (value >> 8) & 0xF, (value >> 4) & 0xF, value & 0xF)
}

//TODO: To macro
fn u8_to_binary_spaced(value: u8) -> String {
    format!("{:04b} {:04b}", (value >> 4) & 0xF, value & 0xF)
}


fn show_chip8_cpu(chip8_emu: &mut chip8::CPU, ui: &mut Ui) {

    let cpu_state = chip8_emu.get_cpu_state();

    let w = Window::new("Chip-8 Status")
        .position([500.0, 500.0], Condition::FirstUseEver)
        .scroll_bar(false)
        .always_auto_resize(true)
        .resizable(false);

    w.build(ui, || {
        ui.columns(1, "General Registers", true);
        ui.separator();

        ui.text_colored(CYAN, "    PC");
        ui.same_line();
        ui.text(format!("= ${:04X}", cpu_state.program_counter as u16));
        ui.text(u16_to_binary_spaced(cpu_state.program_counter as u16));

        ui.text_colored(CYAN, "     I");
        ui.same_line();
        ui.text(format!("= ${:04X}", cpu_state.index_reg as u16)); 
        ui.text(u16_to_binary_spaced(cpu_state.index_reg as u16));

        ui.text_colored(CYAN, "    SP");
        ui.same_line();
        ui.text(format!("= ${:04X}", cpu_state.stack_pointer as u16));
        ui.text(u16_to_binary_spaced(cpu_state.stack_pointer as u16));

        ui.columns(2, "Registers", true);
        ui.separator();

        for idx in 0..cpu_state.registers.len() {
            ui.text_colored(YELLOW, format!(" V{:1X}", idx));
            ui.same_line();
            ui.text(format!("= ${:02X} ", cpu_state.registers[idx]));
            ui.text(u8_to_binary_spaced(cpu_state.registers[idx]));
            ui.next_column();
        }

        ui.columns(1, "rest", true);
        ui.separator();

        ui.text_colored(CYAN, "    DT");
        ui.same_line();
        ui.text(format!("= ${:02X}", cpu_state.delay_timer));
        ui.text(format!("    {}", u8_to_binary_spaced(cpu_state.delay_timer)));

        ui.next_column();

        ui.text_colored(CYAN, "    ST");
        ui.same_line();
        ui.text(format!("= ${:02X}", cpu_state.sound_timer));
        ui.text(format!("    {}", u8_to_binary_spaced(cpu_state.sound_timer)));
    });
}

fn show_memory_disassembled(chip8_emu: &chip8::CPU, ui: &mut Ui) {
    let w = Window::new("Memory Disassembled")
        .position([600.0, 600.0], Condition::FirstUseEver)
        .scroll_bar(false)
        .resizable(true);

    w.build(ui, || {

        let pc = chip8_emu.get_cpu_state().program_counter as u16;
        for address in (pc .. (pc + 50)).step_by(2) {
            let op_info = chip8_emu.disassemble(address);

            ui.text_colored(CYAN, format!("${:04X}", op_info.address));
            ui.same_line();
            ui.text_colored(GREY, format!("${:04X}", op_info.opcode));
            ui.same_line();
            ui.text_colored(RED, if pc == address { "->" } else {"  "});
            ui.same_line();
            ui.text_colored(GREEN, op_info.name);
        }
    });
}

fn show_memory(chip8_emu: &chip8::CPU, ui: &mut Ui) {
    let w = Window::new("Memory")
        .position([300.0, 10.0], Condition::FirstUseEver)
        .scroll_bar(true)
        .size([750.0, 300.0], Condition::FirstUseEver)
        .resizable(false);
    
    w.build (ui, || {
        let memory = chip8_emu.get_memory();
        let mut address_ini: usize = 0;
        let mut address_fin: usize = if memory.len() < 16 { memory.len() } else { 16 };

        while address_ini < memory.len() {
            ui.text(format!("{:04X}:", address_ini as u16));

            for data in &memory[address_ini .. address_fin] {
                ui.same_line();
                ui.text_colored(WHITE, 
                                format!(" {:02X}", data));//memory[address]));
            }

            ui.same_line();
            let mut pos = ui.cursor_pos();
            for data in &memory[address_ini .. address_fin] {
                //ui.same_line();
                //ui.set_cursor_pos(pos);
                ui.same_line_with_spacing(pos[0], 10.0);
                ui.text(format!("{}", if *data < 32 || *data >= 128 { '.' } else { *data as char }));
                pos[0] += 10.0;
            }

            address_ini = address_fin;
            address_fin = if address_ini + 16 < memory.len() { address_ini + 16 } else { memory.len() };
        }
    });
}
