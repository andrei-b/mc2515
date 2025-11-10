#include "xl2515.h"
#include "hardware/spi.h"
#include <string.h>

#define XL2515_SPI_PORT spi1
#define XL2515_SCLK_PIN 10
#define XL2515_MOSI_PIN 11
#define XL2515_MISO_PIN 12
#define XL2515_CS_PIN 9
#define XL2515_INT_PIN 8


bool g_xl2515_recv_flag = false;
static void xl2515_write_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t buf[len + 2];
    buf[0] = CAN_WRITE;
    buf[1] = reg;
    memcpy(buf + 2, data, len);
    gpio_put(XL2515_CS_PIN, 0);
    spi_write_blocking(XL2515_SPI_PORT, buf, len + 2);
    gpio_put(XL2515_CS_PIN, 1);
}

static void xl2515_read_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t buf[2];
    buf[0] = CAN_READ;
    buf[1] = reg;
    gpio_put(XL2515_CS_PIN, 0);
    spi_write_blocking(XL2515_SPI_PORT, buf, 2);
    spi_read_blocking(XL2515_SPI_PORT, 0, data, len);
    gpio_put(XL2515_CS_PIN, 1);
}

static void xl2515_write_reg_byte(uint8_t reg, uint8_t byte)
{
    uint8_t cmd = CAN_WRITE;
    gpio_put(XL2515_CS_PIN, 0);
    spi_write_blocking(XL2515_SPI_PORT, &cmd, 1);
    spi_write_blocking(XL2515_SPI_PORT, &reg, 1);
    spi_write_blocking(XL2515_SPI_PORT, &byte, 1);
    gpio_put(XL2515_CS_PIN, 1);
}

static uint8_t xl2515_read_reg_byte(uint8_t reg)
{
    uint8_t cmd = CAN_READ;
    uint8_t data = 0;
    gpio_put(XL2515_CS_PIN, 0);
    spi_write_blocking(XL2515_SPI_PORT, &cmd, 1);
    spi_write_blocking(XL2515_SPI_PORT, &reg, 1);
    // spi_write_blocking(XL2515_SPI_PORT, &byte, 1);
    spi_read_blocking(XL2515_SPI_PORT, 0, &data, 1);
    gpio_put(XL2515_CS_PIN, 1);
    return data;
}

uint8_t xl2515_get_eflg(void)
{
    return xl2515_read_reg_byte(EFLG);
}

void xl2515_reset(void)
{
    uint8_t buf = CAN_RESET;
    gpio_put(XL2515_CS_PIN, 0);
    spi_write_blocking(XL2515_SPI_PORT, &buf, 1);
    gpio_put(XL2515_CS_PIN, 1);
}

void gpio_callback(uint gpio, uint32_t events)
{
    if (events & GPIO_IRQ_EDGE_FALL)
    {
        // printf("xl2515 recv data done!\r\n");
        g_xl2515_recv_flag = true;
    }
}

// --- internal: pack 11-bit ID into MCP2515 SIDH/SIDL ---
static void id_to_regs(uint16_t id, uint8_t *sidh, uint8_t *sidl) {
    *sidh = (id >> 3) & 0xFF;
    *sidl = (id & 0x07) << 5;  // SID2..0 in bits 7..5
}

void xl2515_init(xl2515_rate_kbps_t rate_kbps)
{
    // CNF1, CNF2, CNF3 values for different bitrates @ your oscillator
    static const uint8_t can_rate_arr[10][3] = {
        {0xA7, 0xBF, 0x07},
        {0x31, 0xA4, 0x04},
        {0x18, 0xA4, 0x04},
        {0x09, 0xA4, 0x04},
        {0x04, 0x9E, 0x03},
        {0x03, 0x9E, 0x03},
        {0x01, 0x1E, 0x03},
        {0x00, 0x9E, 0x03},
        {0x00, 0x92, 0x02},
        {0x00, 0x82, 0x02}
    };

    // --- SPI + GPIO setup ----------------------------------------------------
    spi_init(XL2515_SPI_PORT, 10 * 1000 * 1000);

    gpio_set_function(XL2515_SCLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(XL2515_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(XL2515_MISO_PIN, GPIO_FUNC_SPI);

    gpio_init(XL2515_CS_PIN);
    gpio_set_dir(XL2515_CS_PIN, GPIO_OUT);
    gpio_put(XL2515_CS_PIN, 1);

    gpio_init(XL2515_INT_PIN);
    gpio_set_dir(XL2515_INT_PIN, GPIO_IN);
    gpio_pull_up(XL2515_INT_PIN);

    // INT on both edges, your callback sets g_xl2515_recv_flag, etc.
    gpio_set_irq_enabled_with_callback(
        XL2515_INT_PIN,
        GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE,
        true,
        gpio_callback
    );

    // --- Reset MCP2515 -------------------------------------------------------
    xl2515_reset();
    sleep_ms(10);

    // Put into CONFIG mode explicitly (just to be safe)
    xl2515_write_reg_byte(CANCTRL, REQOP_CONFIG | CLKOUT_ENABLED);
    sleep_ms(1);

    // --- Bit timing (CNF1..3) -----------------------------------------------
    xl2515_write_reg_byte(CNF1, can_rate_arr[rate_kbps][0]);
    xl2515_write_reg_byte(CNF2, can_rate_arr[rate_kbps][1]);
    xl2515_write_reg_byte(CNF3, can_rate_arr[rate_kbps][2]);

    // --- TX buffer 0 default setup (not strictly required for RX) -----------
    // Example: standard ID 0x7FF, RTR=0, DLC=8, TXP=0
    xl2515_write_reg_byte(TXB0SIDH, 0xFF);          // SID[10:3]
    xl2515_write_reg_byte(TXB0SIDL, 0xE0);          // SID[2:0] in bits 7..5, EXIDE=0
    xl2515_write_reg_byte(TXB0DLC,  0x40 | DLC_8);  // 0x40 = TXRTR? (depends on your defines)

    // --- RX configuration: use BOTH buffers, accept all ---------------------
    //
    // Easiest: masks = 0x00000000, filters = 0x00000000 → accept all IDs.
    // Then configure RXB0/RXB1 to use mask/filter and enable rollover.

    // Masks: all "don't care"
    xl2515_write_reg_byte(RXM0SIDH, 0x00);
    xl2515_write_reg_byte(RXM0SIDL, 0x00);
    xl2515_write_reg_byte(RXM1SIDH, 0x00);
    xl2515_write_reg_byte(RXM1SIDL, 0x00);

    // Filters: 0 (since mask=0, they don't really matter)
    xl2515_write_reg_byte(RXF0SIDH, 0x00);
    xl2515_write_reg_byte(RXF0SIDL, 0x00);
    xl2515_write_reg_byte(RXF1SIDH, 0x00);
    xl2515_write_reg_byte(RXF1SIDL, 0x00);
    xl2515_write_reg_byte(RXF2SIDH, 0x00);
    xl2515_write_reg_byte(RXF2SIDL, 0x00);
    xl2515_write_reg_byte(RXF3SIDH, 0x00);
    xl2515_write_reg_byte(RXF3SIDL, 0x00);
    xl2515_write_reg_byte(RXF4SIDH, 0x00);
    xl2515_write_reg_byte(RXF4SIDL, 0x00);
    xl2515_write_reg_byte(RXF5SIDH, 0x00);
    xl2515_write_reg_byte(RXF5SIDL, 0x00);

    // RXB0: accept all (RXM1:RXM0 = 11), enable rollover (BUKT=1)
    // RXB0CTRL bits: 7:6 RXM1:0, bit2 BUKT
    xl2515_write_reg_byte(RXB0CTRL, 0x64);  // 0b0110_0100 → accept all, rollover to RXB1

    // RXB1: accept all (RXM1:RXM0 = 11)
    xl2515_write_reg_byte(RXB1CTRL, 0x60);  // 0b0110_0000

    // Optional: preset DLC if you ever read it as default somewhere
    xl2515_write_reg_byte(RXB0DLC, DLC_8);
    xl2515_write_reg_byte(RXB1DLC, DLC_8);

    // --- Interrupts ---------------------------------------------------------
    xl2515_write_reg_byte(CANINTF, 0x00);   // clear all interrupt flags
    xl2515_write_reg_byte(CANINTE, 0x03);   // enable RX0IE + RX1IE

    // --- Switch to NORMAL mode ----------------------------------------------
    xl2515_write_reg_byte(CANCTRL, REQOP_NORMAL | CLKOUT_ENABLED);
    sleep_ms(1);

    uint8_t stat = xl2515_read_reg_byte(CANSTAT);
    if ((stat & 0xE0) != OPMODE_NORMAL) {
        printf("Failed to enter NORMAL mode, CANSTAT=0x%02X\r\n", stat);
        // Try once more
        xl2515_write_reg_byte(CANCTRL, REQOP_NORMAL | CLKOUT_ENABLED);
        sleep_ms(1);
        stat = xl2515_read_reg_byte(CANSTAT);
        if ((stat & 0xE0) != OPMODE_NORMAL) {
            printf("Still not in NORMAL mode, CANSTAT=0x%02X\r\n", stat);
        }
    }
}


void xl2515_send(uint32_t can_id, uint8_t *data, uint8_t len)
{
    if (len > 8) {
        len = 8;   // CAN max
    }

    // Try to find a free TX buffer: TXB0, TXB1, TXB2
    uint8_t tx_ctrl_addr[3] = { TXB0CTRL, TXB1CTRL, TXB2CTRL };
    uint8_t tx_sidh_addr[3] = { TXB0SIDH, TXB1SIDH, TXB2SIDH };
    uint8_t tx_sidl_addr[3] = { TXB0SIDL, TXB1SIDL, TXB2SIDL };
    uint8_t tx_eid8_addr[3] = { TXB0EID8, TXB1EID8, TXB2EID8 };
    uint8_t tx_eid0_addr[3] = { TXB0EID0, TXB1EID0, TXB2EID0 };
    uint8_t tx_dlc_addr[3]  = { TXB0DLC,  TXB1DLC,  TXB2DLC  };
    uint8_t tx_d0_addr[3]   = { TXB0D0,   TXB1D0,   TXB2D0   };

    uint8_t chosen = 0xFF;
    uint8_t dly = 0;

    while (chosen == 0xFF && dly < 50) {
        // Scan all three TX buffers for a free one
        for (uint8_t i = 0; i < 3; ++i) {
            uint8_t ctrl = xl2515_read_reg_byte(tx_ctrl_addr[i]);
            if ((ctrl & 0x08u) == 0) {    // TXREQ bit clear → buffer free
                chosen = i;
                break;
            }
        }

        if (chosen == 0xFF) {
            // All busy → wait a bit and try again
            sleep_ms(1);
            dly++;
        }
    }

    if (chosen == 0xFF) {
        // No buffer free within timeout → drop the frame or handle error
        // You can add an error counter here if you like.
        return;
    }

    // --- Program the chosen TX buffer --------------------------------------

    // Standard 11-bit ID:
    // SIDH = ID[10:3], SIDL[7:5] = ID[2:0], EXIDE=0, EID bits zero
    xl2515_write_reg_byte(tx_sidh_addr[chosen], (can_id >> 3) & 0xFFu);
    xl2515_write_reg_byte(tx_sidl_addr[chosen], (uint8_t)((can_id & 0x07u) << 5));

    // Extended ID bytes not used for standard frames
    xl2515_write_reg_byte(tx_eid8_addr[chosen], 0x00);
    xl2515_write_reg_byte(tx_eid0_addr[chosen], 0x00);

    // DLC
    xl2515_write_reg_byte(tx_dlc_addr[chosen], len & 0x0Fu);

    // Data bytes
    xl2515_write_reg(tx_d0_addr[chosen], data, len);

    // --- Request to send ----------------------------------------------------
    //
    // Option 1: set TXREQ bit in TXBnCTRL
    xl2515_write_reg_byte(tx_ctrl_addr[chosen],
                          xl2515_read_reg_byte(tx_ctrl_addr[chosen]) | 0x08u);

    // Option 2 (alternative): use RTS SPI command (0x81/0x82/0x84)
    // if you have an SPI helper for that.
}


bool xl2515_recv(uint32_t can_id, uint8_t *data, uint8_t *len)
{
    if (g_xl2515_recv_flag == false)
    {
        return false;
    }
    g_xl2515_recv_flag = false;

    xl2515_write_reg_byte(RXB0SIDH, (can_id >> 3) & 0XFF);
    xl2515_write_reg_byte(RXB0SIDL, (can_id & 0x07) << 5);
    // uint8_t CAN_RX_Buf[];
    while (1)
    {
        if (xl2515_read_reg_byte(CANINTF) & 0x01)
        {
            *len = xl2515_read_reg_byte(RXB0DLC);
            // printf("len = %d\r\n", len);
            for (uint8_t i = 0; i < *len; i++)
            {
                data[i] = xl2515_read_reg_byte(RXB0D0 + i);
                // printf("rx buf =%d\r\n",CAN_RX_Buf[i]);
            }
            break;
        }
    }

    xl2515_write_reg_byte(CANINTF, 0);
    xl2515_write_reg_byte(CANINTE, 0x01);  // enable
    xl2515_write_reg_byte(RXB0SIDH, 0x00); // clean
    xl2515_write_reg_byte(RXB0SIDL, 0x60);
    return true;
}

bool xl2515_recv_any(uint32_t *can_id, uint8_t *data, uint8_t *len)
{
    // No INT from MCP2515 → nothing to do
    if (!g_xl2515_recv_flag) {
        return false;
    }
    g_xl2515_recv_flag = false;

    while (1) {
        uint8_t canintf = xl2515_read_reg_byte(CANINTF);

        // Check RX0 and RX1 flags
        if (canintf & (0x01u | 0x02u)) {  // RX0IF (bit0) or RX1IF (bit1)
            uint8_t base_addr;
            uint8_t clear_mask;

            if (canintf & 0x01u) {
                // RXB0 has a frame
                base_addr  = RXB0SIDH;
                clear_mask = (uint8_t)~0x01u;   // clear only RX0IF
            } else {
                // RXB1 has a frame
                base_addr  = RXB1SIDH;
                clear_mask = (uint8_t)~0x02u;   // clear only RX1IF
            }

            // --- Read CAN ID (11-bit standard frame) ---
            uint8_t sidh = xl2515_read_reg_byte(base_addr + 0); // SIDH
            uint8_t sidl = xl2515_read_reg_byte(base_addr + 1); // SIDL

            *can_id = ((uint32_t)sidh << 3) | (sidl >> 5);

            // --- Read DLC ---
            uint8_t dlc = xl2515_read_reg_byte(base_addr + 4) & 0x0Fu;
            *len = dlc;

            // --- Read DATA bytes ---
            for (uint8_t i = 0; i < dlc; i++) {
                data[i] = xl2515_read_reg_byte((uint8_t)(base_addr + 5 + i));
            }

            // Clear only the flag for the buffer we just read
            uint8_t new_canintf = xl2515_read_reg_byte(CANINTF) & clear_mask;
            xl2515_write_reg_byte(CANINTF, new_canintf);

            // Re-enable RX interrupts for both buffers (RX0IE + RX1IE)
            xl2515_write_reg_byte(CANINTE, 0x03u);

            return true;
        }

        // If we get here: g_xl2515_recv_flag said "yes", but no RX flags yet.
        // Loop until MCP2515 updates CANINTF.
    }
}

xl2515_error_t xl2515_get_error_stats(void)
{
    xl2515_error_t e;
    e.tec  = xl2515_read_reg_byte(TEC);
    e.rec  = xl2515_read_reg_byte(REC);
    e.eflg = xl2515_read_reg_byte(EFLG);
    return e;
}

void xl2515_print_error_stats(void)
{
    xl2515_error_t e = xl2515_get_error_stats();

    printf("CAN Errors: TEC=%u  REC=%u  EFLG=0x%02X\r\n", e.tec, e.rec, e.eflg);

    if (e.eflg & 0x80) printf("  RX1OVR: RX buffer 1 overflowed\r\n");
    if (e.eflg & 0x40) printf("  RX0OVR: RX buffer 0 overflowed\r\n");
    if (e.eflg & 0x20) printf("  TXBO:   Bus-off (CAN wires missing or severe error)\r\n");
    if (e.eflg & 0x10) printf("  TXEP:   TX error-passive\r\n");
    if (e.eflg & 0x08) printf("  RXEP:   RX error-passive\r\n");
    if (e.eflg & 0x04) printf("  TXWAR:  TX warning (TEC >= 96)\r\n");
    if (e.eflg & 0x02) printf("  RXWAR:  RX warning (REC >= 96)\r\n");
    if (e.eflg & 0x01) printf("  EWARN:  At least one warning active\r\n");

    // Optionally check bus-off recovery
    if (e.eflg & 0x20) {
        printf("→ MCP2515 is BUS-OFF; consider resetting it.\r\n");
    }
}

static void xl2515_encode_std_id(uint16_t id, uint8_t *sidh, uint8_t *sidl)
{
    id &= 0x7FFu;                    // 11 bits
    *sidh = (uint8_t)(id >> 3);      // bits 10..3
    *sidl = (uint8_t)((id & 0x07u) << 5);  // bits 2..0 into 7..5, EXIDE=0, EID=0
}

void xl2515_set_filter_mask_std(uint8_t filter_index, uint16_t id, uint16_t mask)
{
    uint8_t f_sidh_addr = 0;
    uint8_t f_sidl_addr = 0;
    uint8_t m_sidh_addr = 0;
    uint8_t m_sidl_addr = 0;

    // Select filter SIDH/SIDL address
    switch (filter_index) {
    case 0:
        f_sidh_addr = RXF0SIDH;
        f_sidl_addr = RXF0SIDL;
        break;
    case 1:
        f_sidh_addr = RXF1SIDH;
        f_sidl_addr = RXF1SIDL;
        break;
    case 2:
        f_sidh_addr = RXF2SIDH;
        f_sidl_addr = RXF2SIDL;
        break;
    case 3:
        f_sidh_addr = RXF3SIDH;
        f_sidl_addr = RXF3SIDL;
        break;
    case 4:
        f_sidh_addr = RXF4SIDH;
        f_sidl_addr = RXF4SIDL;
        break;
    case 5:
        f_sidh_addr = RXF5SIDH;
        f_sidl_addr = RXF5SIDL;
        break;
    default:
        // invalid filter index → ignore
        return;
    }

    // Select mask for this filter:
    //   RXF0, RXF1, RXF2 use mask 0 (RXM0)
    //   RXF3, RXF4, RXF5 use mask 1 (RXM1)
    if (filter_index <= 2) {
        m_sidh_addr = RXM0SIDH;
        m_sidl_addr = RXM0SIDL;
    } else {
        m_sidh_addr = RXM1SIDH;
        m_sidl_addr = RXM1SIDL;
    }

    // Encode ID and mask into SIDH/SIDL
    uint8_t f_sidh, f_sidl;
    uint8_t m_sidh, m_sidl;

    xl2515_encode_std_id(id,   &f_sidh, &f_sidl);
    xl2515_encode_std_id(mask, &m_sidh, &m_sidl);

    // Write filter
    xl2515_write_reg_byte(f_sidh_addr, f_sidh);
    xl2515_write_reg_byte(f_sidl_addr, f_sidl);

    // Write mask
    xl2515_write_reg_byte(m_sidh_addr, m_sidh);
    xl2515_write_reg_byte(m_sidl_addr, m_sidl);
}


void xl2515_clear_filters(void) {
    uint8_t clear_data[4] = {0x00, 0x00, 0x00, 0x00}; // Data to clear filters (4 bytes)

    // Clear all RX filters (RXF0 to RXF5)
    xl2515_write_reg(RXF0SIDH, clear_data, 4);
    xl2515_write_reg(RXF1SIDH, clear_data, 4);
    xl2515_write_reg(RXF2SIDH, clear_data, 4);
    xl2515_write_reg(RXF3SIDH, clear_data, 4);
    xl2515_write_reg(RXF4SIDH, clear_data, 4);
    xl2515_write_reg(RXF5SIDH, clear_data, 4);
}

bool xl2515_eflg_any(uint8_t mask)
{
    return (xl2515_read_reg_byte(EFLG) & mask) != 0;
}

bool xl2515_is_bus_off(void)
{
    return xl2515_eflg_any(EFLG_TXBO);
}

bool xl2515_recover_bus_off(xl2515_rate_kbps_t rate_kbps)
{
    // 1) Reset controller
    xl2515_reset();
    sleep_ms(5);

    // 2) Re-init with last known bitrate
    xl2515_init(rate_kbps);

    // 3) Check if TXBO cleared
    bool ok = !xl2515_is_bus_off();
    if (!ok) {
        printf("xl2515_recover_bus_off(): still BUS-OFF after re-init, EFLG=0x%02X\r\n",
               xl2515_get_eflg());
    }

    return ok;
}

bool xl2515_recv_nb(uint32_t *can_id, uint8_t *data, uint8_t *len)
{
    // Read interrupt flags once
    uint8_t canintf = xl2515_read_reg_byte(CANINTF);

    // Check if RX0IF (bit0) or RX1IF (bit1) is set
    if ((canintf & (0x01u | 0x02u)) == 0) {
        // No receive buffer has data → non-blocking "no frame"
        return false;
    }

    uint8_t base_addr;
    uint8_t clear_mask;

    if (canintf & 0x01u) {
        // RXB0 has a frame
        base_addr  = RXB0SIDH;
        clear_mask = (uint8_t)~0x01u;   // clear RX0IF only
    } else {
        // RXB1 has a frame
        base_addr  = RXB1SIDH;
        clear_mask = (uint8_t)~0x02u;   // clear RX1IF only
    }

    // --- Read ID (standard 11-bit frame) --------------------------------
    uint8_t sidh = xl2515_read_reg_byte(base_addr + 0); // SIDH
    uint8_t sidl = xl2515_read_reg_byte(base_addr + 1); // SIDL

    // ID = SIDH[7:0] as bits 10..3, SIDL[7:5] as bits 2..0
    *can_id = ((uint32_t)sidh << 3) | (sidl >> 5);

    // --- Read DLC --------------------------------------------------------
    uint8_t dlc = xl2515_read_reg_byte(base_addr + 4) & 0x0Fu;
    if (dlc > 8) dlc = 8;
    *len = dlc;

    // --- Read data bytes -------------------------------------------------
    for (uint8_t i = 0; i < dlc; ++i) {
        data[i] = xl2515_read_reg_byte((uint8_t)(base_addr + 5 + i));
    }

    // --- Clear only the RX flag for this buffer --------------------------
    uint8_t new_canintf = xl2515_read_reg_byte(CANINTF) & clear_mask;
    xl2515_write_reg_byte(CANINTF, new_canintf);

    // Re-enable RX interrupts for both buffers (RX0IE + RX1IE)
    xl2515_write_reg_byte(CANINTE, 0x03u);

    return true;
}

// Read current mode (just the OPMOD bits from CANSTAT[7:5])
uint8_t xl2515_get_mode(void)
{
    uint8_t stat = xl2515_read_reg_byte(CANSTAT);
    return (stat & 0xE0u);  // mask bits 7:5
}

// Set mode (REQOP in CANCTRL[7:5]) and wait until CANSTAT matches.
// Returns true on success, false on timeout.
bool xl2515_set_mode(uint8_t mode)
{
    // Keep lower 5 bits (CLKOUT, one-shot, etc.), only change bits 7:5
    uint8_t canctrl = xl2515_read_reg_byte(CANCTRL);
    canctrl &= ~0xE0u;      // clear mode bits
    canctrl |= (mode & 0xE0u);
    xl2515_write_reg_byte(CANCTRL, canctrl);

    // Wait until CANSTAT[7:5] == requested mode (with small timeout)
    const uint8_t max_tries = 50;
    for (uint8_t i = 0; i < max_tries; ++i) {
        uint8_t stat = xl2515_read_reg_byte(CANSTAT) & 0xE0u;
        if (stat == (mode & 0xE0u)) {
            return true;
        }
        sleep_ms(1);
    }
    return false;   // timeout
}



bool xl2515_set_normal_mode(void)
{
    return xl2515_set_mode(REQOP_NORMAL);
}

bool xl2515_set_loopback_mode(void)
{
    return xl2515_set_mode(REQOP_LOOPBACK);
}

bool xl2515_set_listen_only_mode(void)
{
    return xl2515_set_mode(REQOP_LISTEN);
}

bool xl2515_set_sleep_mode(void)
{
    return xl2515_set_mode(REQOP_SLEEP);
}

bool xl2515_set_config_mode(void)
{
    return xl2515_set_mode(REQOP_CONFIG);
}