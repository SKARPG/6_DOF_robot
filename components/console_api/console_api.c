/**
 * @file console_api.c
 * @author JanG175
 * @brief 6-DOF ROBOT MOTOR CONSOLE API
 * 
 * @copyright All rigths reserved (R) 2023
 */

#include <stdio.h>
#include "console_api.h"

static const char* TAG = "console_demo";

// struct for servo_move command arguments
static struct {
    struct arg_int* DOF;
    struct arg_dbl* pos;
    struct arg_int* speed;
    struct arg_end* end;
} servo_move_args;


/**
 * @brief command for moving a single DOF
 * 
 * @param argc number of arguments
 * @param argv array of arguments
 * @return 0 - success, 1 - error
 */
static int cmd_servo_move(int argc, char** argv)
{
    int nerrors = arg_parse(argc, argv, (void**)&servo_move_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, servo_move_args.end, argv[0]);
        return 1;
    }

    assert(servo_move_args.DOF->count == 1);
    assert(servo_move_args.pos->count == 1);
    assert(servo_move_args.speed->count == 1);
    const uint8_t DOF = servo_move_args.DOF->ival[0];
    const float pos = servo_move_args.pos->dval[0];
    const uint8_t speed = servo_move_args.speed->ival[0];

    if (DOF != 1 && DOF != 2 && DOF != 3 && DOF != 4 && DOF != 5 && DOF != 6)
    {
        printf("wrong DOF number!\n");
        return 1;
    }

    if (speed > 100)
    {
        printf("wrong speed!\n");
        return 1;
    }

    printf("moving servo: %d, to position: %f, with speed: %u ...\n", DOF, pos, speed);

    single_DOF_move(DOF - 1, pos, speed);
    wait_for_motors_stop();

    printf("done!\n");

    return 0;
}


/**
 * @brief register servo_move command
 * 
 */
static void register_servo_move()
{
    servo_move_args.DOF = arg_int1(NULL, NULL, "<1|2|3|4|5|6>", "number of DOF (1 - 6)");
    servo_move_args.pos = arg_dbl1(NULL, NULL, "<float>", "desired position in degrees");
    servo_move_args.speed = arg_int1(NULL, NULL, "<0-100>", "percent of speed (0 - 100 %)");
    servo_move_args.end = arg_end(4);

    const esp_console_cmd_t cmd = {
        .command = "servo_move",
        .help = "Move servo to a given position",
        .hint = NULL,
        .func = &cmd_servo_move,
        .argtable = &servo_move_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}


/**
 * @brief iniiialize non-volatile storage
 * 
 */
static void init_nvs()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK(err);
}


/**
 * @brief initialize filesystem
 * 
 */
static void init_filesystem()
{
    static wl_handle_t wl_handle;
    const esp_vfs_fat_mount_config_t mount_config = {
        .max_files = 4,
        .format_if_mount_failed = true
    };

    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(CONSOLE_MOUNT_PATH, "storage", &mount_config, &wl_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)\n", esp_err_to_name(err));
        return;
    }
}


/**
 * @brief initialize console
 * 
 */
static void init_console()
{
    // drain stdout before reconfiguring it
    fflush(stdout);
    fsync(fileno(stdout));

    // disable buffering on stdin
    setvbuf(stdin, NULL, _IONBF, 0);

    // minicom, screen, idf_monitor send CR when ENTER key is pressed
    esp_vfs_dev_uart_port_set_rx_line_endings(CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    // move the caret to the beginning of the next line on '\n'
    esp_vfs_dev_uart_port_set_tx_line_endings(CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);

    // configure UART
    const uart_config_t uart_config = {
        .baud_rate = CONSOLE_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CONSOLE_UART_NUM, &uart_config));

    // tell VFS to use UART driver
    esp_vfs_dev_uart_use_driver(CONSOLE_UART_NUM);

    // initialize console
    esp_console_config_t console_config = {
        .max_cmdline_args = 8,
        .max_cmdline_length = 256,
        .hint_color = atoi(LOG_COLOR_CYAN)
    };
    ESP_ERROR_CHECK(esp_console_init(&console_config));

    // configure linenoise line completion library

    // enable multiline editing (if not set, long commands will scroll within single line)
    linenoiseSetMultiLine(1);

    // tell linenoise where to get command completions and hints
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback*)&esp_console_get_hint);

    // set command history size
    linenoiseHistorySetMaxLen(100);

    // set command maximum length
    linenoiseSetMaxLineLen(console_config.max_cmdline_length);

    // don't return empty lines
    linenoiseAllowEmpty(false);

    // load command history from filesystem
    linenoiseHistoryLoad(CONSOLE_HISTORY_PATH);
}


/**
 * @brief start console main loop
 * 
 */
void console_api_start()
{
    init_nvs();

    init_filesystem();
    ESP_LOGI(TAG, "Command history enabled!");

    init_console();

    // register commands
    register_servo_move();

    // prompt to be printed before each line (this can be customized, made dynamic, etc.)
    const char* prompt = LOG_COLOR_I PROMPT_STR "> " LOG_RESET_COLOR;

    printf("\n"
        "This is the 6-DOF robotic arm console.\n"
        "Type 'help' to get the list of commands.\n"
        "Use UP/DOWN arrows to navigate through command history.\n"
        "Press TAB when typing command name to auto-complete.\n"
        "Press Enter or Ctrl+C will terminate the console environment.\n");

    // figure out if the terminal supports escape sequences
    int probe_status = linenoiseProbe();
    if (probe_status)
    {
        printf("\n"
            "Your terminal application does not support escape sequences.\n"
            "Line editing and history features are disabled.\n"
            "On Windows, try using Putty instead.\n");

        linenoiseSetDumbMode(1);

        prompt = PROMPT_STR "> ";
    }

    // main loop
    while (1)
    {
        // get a line using linenoise (the line is returned when ENTER is pressed)
        char* line = linenoise(prompt);
        if (line == NULL) // break on EOF or error
            break;

        // add the command to the history if not empty
        if (strlen(line) > 0)
        {
            linenoiseHistoryAdd(line);
            // save command history to filesystem
            linenoiseHistorySave(CONSOLE_HISTORY_PATH);
        }

        // try to run the command
        int ret;
        esp_err_t err = esp_console_run(line, &ret);
        if (err == ESP_ERR_NOT_FOUND)
            printf("Unrecognized command\n");
        else if (err == ESP_ERR_INVALID_ARG)
            printf("Command was empty\n");
        else if (err == ESP_OK && ret != ESP_OK)
            printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(err));
        else if (err != ESP_OK)
            printf("Internal error: %s\n", esp_err_to_name(err));

        // linenoise allocates line buffer on the heap, so need to free it
        linenoiseFree(line);
    }

    ESP_LOGE(TAG, "Error or end-of-input, terminating console");
    esp_console_deinit();
}