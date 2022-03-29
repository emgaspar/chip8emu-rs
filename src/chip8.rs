use rand::Rng;

const INIT_PROGRAM_ADDRESS: usize = 0x200;

/// chip8 original screen size
pub const SCREEN_SIZE: (usize, usize) = (64, 32);

const TOT_FONT_SIZE: usize = 80;

/// predefined font sprites
const FONT_DATA: [u8; TOT_FONT_SIZE] = [
    0xF0, 0x90, 0x90, 0x90, 0xF0, // 0
    0x20, 0x60, 0x20, 0x20, 0x70, // 1
    0xF0, 0x10, 0xF0, 0x80, 0xF0, // 2
    0xF0, 0x10, 0xF0, 0x10, 0xF0, // 3
    0x90, 0x90, 0xF0, 0x10, 0x10, // 4
    0xF0, 0x80, 0xF0, 0x10, 0xF0, // 5
    0xF0, 0x80, 0xF0, 0x90, 0xF0, // 6
    0xF0, 0x10, 0x20, 0x40, 0x40, // 7
    0xF0, 0x90, 0xF0, 0x90, 0xF0, // 8
    0xF0, 0x90, 0xF0, 0x10, 0xF0, // 9
    0xF0, 0x90, 0xF0, 0x90, 0x90, // A
    0xE0, 0x90, 0xE0, 0x90, 0xE0, // B
    0xF0, 0x80, 0x80, 0x80, 0xF0, // C
    0xE0, 0x90, 0x90, 0x90, 0xE0, // D
    0xF0, 0x80, 0xF0, 0x80, 0xF0, // E
    0xF0, 0x80, 0xF0, 0x80, 0x80, // F
];

pub struct DisassembleRecord {
    pub address : u16,
    pub opcode  : u16,
    pub name    : String,
}

impl DisassembleRecord{
    pub fn new() -> DisassembleRecord {
        DisassembleRecord {
            address: 0,
            opcode: 0,
            name: String::new()
        }
    }
}

pub struct CPUState {
    pub registers : [u8; 16],
    pub index_reg : usize,
    pub program_counter : usize,
    pub stack_pointer : usize,
    pub stack : [usize; 16],
    pub delay_timer : u8,
    pub sound_timer : u8,
}

impl CPUState {
    pub fn new() -> CPUState {
        CPUState { 
            registers: [0; 16], 
            index_reg: 0, 
            program_counter: INIT_PROGRAM_ADDRESS, 
            stack_pointer: 0, 
            stack: [0; 16], 
            delay_timer: 0, 
            sound_timer: 0 
        }        
    }
}

pub struct CPU {
    memory : [u8; 0x1000],
    registers : [u8; 16],
    index_reg : usize,
    program_counter : usize,
    stack_pointer : usize,
    stack : [usize; 16],
    delay_timer : u8,
    sound_timer : u8,

    screen : [bool; SCREEN_SIZE.1 * SCREEN_SIZE.0],

    keyboard : [bool; 16],
}

impl CPU {
    pub fn new() -> CPU {
        let mut my_cpu = CPU {
            memory : [0; 0x1000],
            registers: [0; 16],
            index_reg: 0,
            program_counter: INIT_PROGRAM_ADDRESS,
            stack_pointer: 0,
            stack: [0; 16],
            delay_timer: 0,
            sound_timer: 0,
            screen : [false; SCREEN_SIZE.1 * SCREEN_SIZE.0],
            keyboard : [false; 16],
        };

        my_cpu.memory[..TOT_FONT_SIZE].copy_from_slice(&FONT_DATA);
        my_cpu
    }

    pub fn reset(&mut self) {
        self.registers = [0; 16];
        self.index_reg = 0;
        self.program_counter = INIT_PROGRAM_ADDRESS;
        self.stack_pointer = 0;
        self.stack = [0; 16];
        self.delay_timer = 0;
        self.sound_timer = 0;
        self.screen = [false; SCREEN_SIZE.1 * SCREEN_SIZE.0];
        self.keyboard = [false; 16];
    }

    pub fn load_rom(&mut self, rom: &[u8]){
        let start = INIT_PROGRAM_ADDRESS as usize;
        let end = (INIT_PROGRAM_ADDRESS as usize) + rom.len();
        self.memory[start..end].copy_from_slice(rom);        
    }

    pub fn get_cpu_state(&self) -> CPUState {
        CPUState {
            registers: self.registers,
            index_reg: self.index_reg,
            program_counter: self.program_counter,
            stack_pointer: self.stack_pointer,
            stack: self.stack,
            delay_timer: self.delay_timer,
            sound_timer: self.sound_timer
        }
    }

    pub fn get_screen(&self) -> &[bool] {
        &self.screen
    }

    pub fn get_memory(&self) -> &[u8] {
        &self.memory
    }

    pub fn run_tick(&mut self) {
        let opcode = self.fetch_opcode(self.program_counter);

        self.program_counter += 2;

        let c: u8 = ((opcode >> 12) & 0xF) as u8;
        let x: u8 = ((opcode >> 8) & 0xF) as u8;
        let y: u8 = ((opcode >> 4) & 0xF) as u8;
        let d: u8 = (opcode & 0xF) as u8;
        let addr: usize = (opcode & 0xFFF) as usize;
        let byte: u8 = (opcode & 0xFF) as u8;
        let nibble: u8 = (opcode & 0xF) as u8;

        match (c, x, y, d) {
            (0x0, _, 0xE, 0xE)  =>  // RET
                self.op_ret(),
            (0x0, _, 0xE, _)    =>  // CLS
                self.op_cls(),
            (0x1, _, _, _)      =>  // JUMP
                self.op_jump(addr),
            (0x2, _, _, _)      =>  // CALL
                self.op_call(addr),
            (0x3, _, _, _)      =>  // SE Reg, Value
                self.op_se_vx_byte(x, byte),
            (0x4, _, _, _)      =>  // SNE Reg, Value
                self.op_sne_vx_byte(x, byte),
            (0x5, _, _, _)      =>  // SE Reg, Reg
                self.op_se_vx_vy(x, y),
            (0x6, _, _, _)      =>  // LD Reg, Value
                self.op_ld_vx_byte(x, byte),
            (0x7, _, _, _)      =>  // ADD Reg, Value
                self.op_add_vx_byte(x, byte),
            (0x8, _, _, 0x0)    =>  // LD Reg, Reg
                self.op_ld_vx_vy(x, y),
            (0x8, _, _, 0x1)    =>  // OR Reg, Reg
                self.op_or_vx_vy(x,y),
            (0x8, _, _, 0x2)    =>  // AND Reg, Reg
                self.op_and_vx_vy(x, y),
            (0x8, _, _, 0x3)    =>  // XOR Reg, Reg
                self.op_xor_vx_vy(x,y),
            (0x8, _, _, 0x4)    =>  // ADD Reg, Reg
                self.op_add_vx_vy(x, y),
            (0x8, _, _, 0x5)    =>  // SUB Reg, Reg
                self.op_sub_vx_vy(x, y),
            (0x8, _, _, 0x6)    =>  // SHR Reg
                self.op_shr_vx(x),
            (0x8, _, _, 0x7)    =>  // SUBN Reg, Reg
                self.op_subn_vx_vy(x, y),
            (0x8, _, _, 0xE)    =>  // SHL Reg
                self.op_shl_vx(x),
            (0x9, _, _, 0x0)    =>  // SNE Reg, Reg
                self.op_sne_vx_vy(x, y),
            (0xA, _, _, _)      =>  // LD I, Addr
                self.op_ld_i_addr(addr as usize),
            (0xB, _, _, _)      =>  // JP V0, Addr
                self.op_jp_v0_addr(addr as usize),
            (0xC, _, _, _)      =>  // RND V0, Value
                self.op_rnd_vx_value(x, byte),
            (0xD, _, _, _)      =>  // DRW V0, Addr
                self.op_drw_vx_vy_nibble(x, y, nibble),
            (0xE, _, 0x9, 0xE)  =>  // Skp Vx
                self.op_skp(x),
            (0xE, _, 0xA, 0x1)  =>  // Sknp Vx
                self.op_sknp(x),
            (0xF, _, 0x0, 0x7)  =>  // LD Vx, DT
                self.op_ld_vx_dt(x),
            (0xF, _, 0x0, 0xA)  =>  // LD Vx, K
                self.op_ld_vx_key(x),
            (0xF, _, 0x1, 0x5)  =>  // LD DT, Vx
                self.op_ld_dt_vx(x),
            (0xF, _, 0x1, 0x8)  =>  // LD ST, Vx
                self.op_ld_st_vx(x),
            (0xF, _, 0x1, 0xE)  =>  // ADD I, Vx
                self.op_add_i_vx(x),
            (0xF, _, 0x2, 0x9)  =>  // LD F, Vx
                self.op_ld_sprite_vx(x),
            (0xF, _, 0x3, 0x3)  =>  // LD B, Vx
                self.op_ld_bcd_vx(x),
            (0xF, _, 0x5, 0x5)  =>  // LD [I], Vx
                self.op_ld_i_addr_vx(x),
            (0xF, _, 0x6, 0x5)  =>  // LD Vx, [I]
                self.op_ld_vx_i_addr(x),
            _                   => 
                todo!("opcode 0x{:04x}", opcode),
        }
    }

    //--------------------------------------------------------------------------
    // Private operations
    //--------------------------------------------------------------------------
    /// Operation which fetch a new opcode
    fn fetch_opcode(&self, pc: usize) -> u16 {
        ((self.memory[pc] as u16) << 8) | (self.memory[pc + 1] as u16)
    }

    pub fn disassemble(&self, address: u16) -> DisassembleRecord {
        let mut instruction = DisassembleRecord::new();
        
        instruction.address = address;
        instruction.opcode = self.fetch_opcode(address as usize);
        
        let c: u8 = ((instruction.opcode >> 12) & 0xF) as u8;
        let vx: u8 = ((instruction.opcode >> 8) & 0xF) as u8;
        let vy: u8 = ((instruction.opcode >> 4) & 0xF) as u8;
        let d: u8 = (instruction.opcode & 0xF) as u8;
        let addr: usize = (instruction.opcode & 0xFFF) as usize;
        let byte: u8 = (instruction.opcode & 0xFF) as u8;
        let nibble: u8 = (instruction.opcode & 0xF) as u8;

        match (c, vx, vy, d) {
            (0x0, 0x0, 0x0, 0x0)  =>  // RET
                instruction.name = String::from("NOP"),
            (0x0, _, 0xE, 0xE)  =>  // RET
                instruction.name = String::from("RET"),
            (0x0, _, 0xE, _)    =>  // CLS
                instruction.name = String::from("CLS"),
            (0x1, _, _, _)      =>  // JUMP
                instruction.name = format!("JP {:03X}", addr),
            (0x2, _, _, _)      =>  // CALL
                instruction.name = format!("CALL {:03X}", addr),
            (0x3, _, _, _)      =>  // SE Reg, Value
                instruction.name = format!("SE V{:1X}, {:2X}", vx, byte),
            (0x4, _, _, _)      =>  // SNE Reg, Value
                instruction.name = format!("SNE V{:1X}, {:2X}", vx, byte),
            (0x5, _, _, _)      =>  // SE Reg, Reg
                instruction.name = format!("SE V{:1X}, {:2X}", vx, vy),
            (0x6, _, _, _)      =>  // LD Reg, Value
                instruction.name = format!("LD V{:1X}, {:X}", vx, byte),
            (0x7, _, _, _)      =>  // ADD Reg, Value
                instruction.name = format!("ADD V{:1X}, {:X}", vx, byte),
            (0x8, _, _, 0x0)    =>  // LD Reg, Reg
                instruction.name = format!("LD V{:1X}, V{:1X}", vx, vy),
            (0x8, _, _, 0x1)    =>  // OR Reg, Reg
                instruction.name = format!("OR V{:1X}, V{:1X}", vx, vy),
            (0x8, _, _, 0x2)    =>  // AND Reg, Reg
                instruction.name = format!("AND V{:1X}, V{:1X}", vx, vy),
            (0x8, _, _, 0x3)    =>  // XOR Reg, Reg
                instruction.name = format!("XOR V{:1X}, V{:1X}", vx, vy),
            (0x8, _, _, 0x4)    =>  // ADD Reg, Reg
                instruction.name = format!("ADD V{:1X}, V{:1X}", vx, vy),
            (0x8, _, _, 0x5)    =>  // SUB Reg, Reg
                instruction.name = format!("SUB V{:1X}, V{:1X}", vx, vy),
            (0x8, _, _, 0x6)    =>  // SHR Reg
                instruction.name = format!("SHR V{:1X}", vx),
            (0x8, _, _, 0x7)    =>  // SUBN Reg, Reg
                instruction.name = format!("SUBN V{:1X}, V{:1X}", vx, vy),
            (0x8, _, _, 0xE)    =>  // SHL Reg
                instruction.name = format!("SHR V{:1X}", vx),
            (0x9, _, _, 0x0)    =>  // SNE Reg, Reg
                instruction.name = format!("SNE V{:1X}, V{:1X}", vx, vy),
            (0xA, _, _, _)      =>  // LD I, Addr
                instruction.name = format!("LD I, {:03X}", addr),
            (0xB, _, _, _)      =>  // JP V0, Addr
                instruction.name = format!("JP V0, {:03X}", addr),
            (0xC, _, _, _)      =>  // RND V0, Value
                instruction.name = format!("RND V{:1X}, {:03X}", vx, byte),
            (0xD, _, _, _)      =>  // DRW V0, Addr
                instruction.name = format!("DRW V{:1X}, V{:1X}, {:1X}", vx, vy, nibble),
            (0xE, _, 0x9, 0xE)  =>  // Skp Vx
                instruction.name = format!("SKP V{:1X}", vx),
            (0xE, _, 0xA, 0x1)  =>  // Sknp Vx
                instruction.name = format!("SKNP V{:1X}", vx),
            (0xF, _, 0x0, 0x7)  =>  // LD Vx, DT
                instruction.name = format!("LD V{:1X}, DT", vx),
            (0xF, _, 0x0, 0xA)  =>  // LD Vx, K
                instruction.name = format!("LD V{:1X}, K", vx),
            (0xF, _, 0x1, 0x5)  =>  // LD DT, Vx
                instruction.name = format!("LD DR, V{:1X}", vx),
            (0xF, _, 0x1, 0x8)  =>  // LD ST, Vx
                instruction.name = format!("LD ST, V{:1X}", vx),
            (0xF, _, 0x1, 0xE)  =>  // ADD I, Vx
                instruction.name = format!("ADD I, V{:1X}", vx),
            (0xF, _, 0x2, 0x9)  =>  // LD F, Vx
                instruction.name = format!("LD F, V{:1X}", vx),
            (0xF, _, 0x3, 0x3)  =>  // LD B, Vx
                instruction.name = format!("LD B, V{:1X}", vx),
            (0xF, _, 0x5, 0x5)  =>  // LD [I], Vx
                instruction.name = format!("LD [I], V{:1X}", vx),
            (0xF, _, 0x6, 0x5)  =>  // LD Vx, [I]
                instruction.name = format!("LD V{:1X}, [I]", vx),
            _                   => 
                instruction.name = String::from("Unknown"),
        }

        instruction
    }
    
    /// Operation: CLS (0x00E0)
    fn op_cls(&mut self) {
        self.screen = [false; SCREEN_SIZE.0 * SCREEN_SIZE.1];
    }

    /// Operation: RET (0x00EE)
    fn op_ret(&mut self) {
        self.program_counter = self.stack[self.stack_pointer];
        self.stack_pointer-=1;
    }

    /// Operation: JP addr (0x1NNN)
    fn op_jump(&mut self, addr: usize) {
        self.program_counter = addr;
    }

    /// Operation: CALL addr (0x2nnn)
    fn op_call(&mut self, addr: usize) {
        self.stack_pointer+=1;
        self.stack[self.stack_pointer] = self.program_counter;
        self.program_counter = addr;
    }

    /// Operation: SE Vx, byte (0x3xkk)
    fn op_se_vx_byte(&mut self, vx: u8, byte: u8) {
        if self.registers[vx as usize] == byte {
            self.program_counter += 2;
        }
    }

    /// Operation: SNE Vx, byte (0x4xkk)
    fn op_sne_vx_byte(&mut self, vx: u8, byte: u8) {
        if self.registers[vx as usize] != byte {
            self.program_counter += 2;
        }
    }
    
    /// Operation: SE Vx, Vy (0x5xy0)
    fn op_se_vx_vy(&mut self, vx: u8, vy: u8) {
        if self.registers[vx as usize] == self.registers[vy as usize] {
            self.program_counter += 2;
        }
    }

    /// Operation: LD Vx, byte (0x6xnn)
    fn op_ld_vx_byte(&mut self, vx: u8, byte: u8) {
        self.registers[vx as usize] = byte;
    }

    /// Operation: ADD Vx, byte (0x7xnn)
    fn op_add_vx_byte(&mut self, vx: u8, byte: u8) {
        let (new_vx, _borrow) = 
            self.registers[vx as usize].overflowing_add(byte);
        self.registers[vx as usize] = new_vx;
    }

    /// Operation: LD Vx, Vy (0x8xy0)
    fn op_ld_vx_vy(&mut self, vx: u8, vy: u8) {
        self.registers[vx as usize] = self.registers[vy as usize];
    }

    /// Operation: OR Vx, Vy (0x8xy1)
    fn op_or_vx_vy(&mut self, vx: u8, vy: u8) {
        self.registers[vx as usize] |= self.registers[vy as usize];
    }

    /// Operation: AND Vx, Vy (0x8xy2)
    fn op_and_vx_vy(&mut self, vx: u8, vy: u8) {
        self.registers[vx as usize] &= self.registers[vy as usize];
    }

    /// Operation: XOR Vx, Vy (0x8xy3)
    fn op_xor_vx_vy(&mut self, vx: u8, vy: u8) {
        self.registers[vx as usize] ^= self.registers[vy as usize];
    }

    /// Operation: ADD Vx, Vy (0x8xy4)
    fn op_add_vx_vy(&mut self, vx: u8, vy: u8) {
        let (new_vx, borrow) = 
            self.registers[vx as usize].overflowing_add(self.registers[vy as usize]);
        let new_vf = if borrow { 0 } else { 1 };

        self.registers[vx as usize] = new_vx;
        self.registers[0xF] = new_vf;
    }

    /// Operation: SUB Vx, Vy (0x8xy5)
    fn op_sub_vx_vy(&mut self, vx: u8, vy: u8) {
        let (new_vx, borrow) = 
            self.registers[vx as usize].overflowing_sub(self.registers[vy as usize]);
        let new_vf = if borrow { 0 } else { 1 };

        self.registers[vx as usize] = new_vx;
        self.registers[0xF] = new_vf;
    }

    /// Operation: SHR Vx (0x8xy6)
    fn op_shr_vx(&mut self, vx: u8) {
        if (self.registers[vx as usize] & 0x1) != 0 {
            self.registers[0xF] = 1;
        }
        else {
            self.registers[0xF] = 0;
        }

        self.registers[vx as usize] >>= 1;
    }

    /// Operation: SUBN Vx, Vy (0x8xy7)
    fn op_subn_vx_vy(&mut self, vx: u8, vy: u8) {
        let (new_vx, borrow) = 
            self.registers[vy as usize].overflowing_sub(self.registers[vx as usize]);
        let new_vf = if borrow { 0 } else { 1 };

        self.registers[vx as usize] = new_vx;
        self.registers[0xF] = new_vf;
    }

    /// Operation: SHL Vx (0x8xyE)
    fn op_shl_vx(&mut self, vx: u8) {
        if (self.registers[vx as usize] & 0x1) != 0 {
            self.registers[0xF] = 1;
        }
        else {
            self.registers[0xF] = 0;
        }

        self.registers[vx as usize] <<= 1;
    }

    /// Operation: SNE Vx, Vy (0x9xy0)
    fn op_sne_vx_vy(&mut self, vx: u8, vy: u8) {
        if self.registers[vx as usize] != self.registers[vy as usize] {
            self.program_counter += 2;
        }
    }

    /// Operation: LD I, addr (0xAnnn)
    fn op_ld_i_addr(&mut self, addr: usize) {
        self.index_reg = addr;
    }

    /// Operation: JP V0, addr (0xBnnn)
    fn op_jp_v0_addr(&mut self, addr: usize) {
        self.program_counter = (self.registers[0] as usize) + addr;
    }

    /// Operation: RND Vx, byte (0xCxkk)
    fn op_rnd_vx_value(&mut self, vx: u8, byte: u8) {
        let rnd: u8 = rand::thread_rng().gen();
        self.registers[vx as usize] = rnd & byte;
    }

    /// Operation: DRW Vx, Vy, nibble (0xDxyn)
    fn op_drw_vx_vy_nibble(&mut self, vx: u8, vy: u8, nibble: u8)
    {
        let initial_col = self.registers[vx as usize] as usize;
        let initial_row = self.registers[vy as usize] as usize;
        let num_bytes = nibble;

        self.registers[0xF] = 0;
        let mut current_row = initial_row;
        for idx in 0..(num_bytes as usize) {
            let mem_value = self.memory[self.index_reg + idx];
            let mut current_col = initial_col;

            for idx_pixel in 0..7 {
                if (mem_value & (0x80 >> idx_pixel)) != 0 {
                    let offset = 
                        ((current_row % SCREEN_SIZE.1) * SCREEN_SIZE.0) + 
                        (current_col % SCREEN_SIZE.0);
                    if self.screen[offset] {
                        self.registers[0xF] = 1;
                        self.screen[offset] = false;
                    } else {
                        self.screen[offset] = true;
                    }
                }
                current_col += 1;
            }
            current_row += 1;
        }
    }

    /// Operation: SKP Vx (0xEx9E)
    fn op_skp(&mut self, key_id: u8) {
        if self.keyboard[key_id as usize] {
            self.program_counter += 2;
        }
    }

    /// Operation: SKNP Vx (0xExA1)
    fn op_sknp(&mut self, key_id: u8) {
        if !self.keyboard[key_id as usize] {
            self.program_counter += 2;
        }
    }

    /// Operation: LD Vx, DT (0xFx07)
    fn op_ld_vx_dt(&mut self, vx: u8) {
        self.registers[vx as usize] = self.delay_timer;
    }

    /// Operation: LD Vx, K (0xFx0A)
    fn op_ld_vx_key(&mut self, vx: u8) {
        
        // If not key pressed, current operation is executed again.
        self.program_counter -= 2;

        for idx in 0..=16 {
            if self.keyboard[idx] {
                self.registers[vx as usize] = idx as u8;
                self.program_counter += 2;
                break;
            }
        }
    }

    /// Operation: LD DT, Vx (0xFx15)
    fn op_ld_dt_vx(&mut self, vx: u8) {
        self.delay_timer = self.registers[vx as usize];
    }

    /// Operation: LD ST, Vx (0xFx18)
    fn op_ld_st_vx(&mut self, vx: u8) {
        self.sound_timer = self.registers[vx as usize];
    }

    /// Operation: ADD I, Vx (0xFx1E)
    fn op_add_i_vx(&mut self, vx: u8) {
        self.index_reg += self.registers[vx as usize] as usize;
    }

    /// Operation: LD F, Vx (0xFx29)
    fn op_ld_sprite_vx(&mut self, vx: u8) {
        self.index_reg = (5 * self.registers[vx as usize]) as usize;
    }

    /// Operation: LD B, Vx (0xFx33)
    fn op_ld_bcd_vx(&mut self, vx: u8) {
        let value: u8 = self.registers[vx as usize]; 
        self.memory[self.index_reg] = value / 100;
        self.memory[self.index_reg + 1] = 
            (value - (100 * self.memory[self.index_reg])) / 10;
        self.memory[self.index_reg + 2] = value % 10;
    }

    /// Operation: LD [I], Vx (0xFx55)
    fn op_ld_i_addr_vx(&mut self, vx: u8) {
        for idx in 0..=(vx as usize){
            self.memory[self.index_reg + idx] = self.registers[idx];
        }
    }

    /// Operation: LD Vx, [I] (0xFx65)
    fn op_ld_vx_i_addr(&mut self, vx: u8) {
        for idx in 0..=(vx as usize) {
            self.registers[idx] = self.memory[self.index_reg + idx];
        }
    }
}