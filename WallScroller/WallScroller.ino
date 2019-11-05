/*
 Name:		WallScroller.ino
 Created:	10/17/2019 10:09:56 AM
 Author:	Benjamin Balogh
*/

// Includes
#include <LEDMatrixDriver.hpp>
#include <eeprom.h>

#include "LEDMatrixData.hpp"
#include "MPU6050Libized.h"

// Constexpr functions
constexpr int HZ_TO_MILLIS(int hz) {
    return 1000 / hz;
}

// Constexprs defines
constexpr auto LEDMATRIX_CS_PIN = 9;  // Chip Select pin of LED matrix PCB

constexpr auto BLOCK_COUNT = 4;      // number of 8x8 pixel matrices;
constexpr auto BLOCK_ROW_COUNT = 8;  // number of rows per block
constexpr auto BLOCK_COL_COUNT = 8;  // number of coloumns per block
constexpr auto LEDMATRIX_WIDTH = BLOCK_COUNT * BLOCK_COL_COUNT;  // pixels in the whole LED matrix
constexpr auto LEDMATRIX_DEFAULT_INTENSITY = 8;  // default LEDMatrix intensity [0-15]
constexpr auto LEDMATRIX_MAX_INTENSITY = 15;     // max LEDMatrix intensity

constexpr auto DEFAULT_SCROLL_INTERVAL = HZ_TO_MILLIS(10); // default ms between display scrolls
constexpr auto MIN_SCROLL_INTERVAL = HZ_TO_MILLIS(40);     // min ms between display scrolls
constexpr auto PLAYER_SHOW_INTERVAL = 60;                  // [ms] player pixel on state duration
constexpr auto PLAYER_HIDE_INTERVAL = 30;                  // [ms] player pixel off state duration

constexpr auto TEXT_SCROLL_INTERVAL = HZ_TO_MILLIS(50);          // ms between any text scrolls
constexpr auto PLAY_MSG_UPDATE_INTERVAL = HZ_TO_MILLIS(10);      // ms between play message updates
constexpr auto PLAYER_POS_UPDATE_INTERVAL = HZ_TO_MILLIS(60);    // ms between player position updates
constexpr auto DEMO_MSG_DISPLAY_INTERVAL = 1700;//1375;                 // [ms] How long should the demo message be displayed
constexpr auto ENDGAME_INSTENSIFY_INTERVAL = HZ_TO_MILLIS(25);   // ms between display intensity changes after game over
constexpr auto ENDGAME_PLAYER_BLINK_INTERVAL = HZ_TO_MILLIS(5);  // ms between player pixel state changes after game over
constexpr auto ENDGAME_AFK_INTERVAL = 30 * 1000;                 // ms after AI control is automatically turned on when in endgame

constexpr auto ENDGAME_INSTENSIFY_CYCLES_COUNT = 3;  // how many cycles should the endgame intensification take place
constexpr auto MIN_WALL_THICKNESS = 1;               // at least how thick the walls should be
constexpr auto MAX_WALL_THICKNESS = 5;               // at most how thick the walls should be
constexpr auto MIN_SPACE_BETWEEN_WALLS = 5;          // at least how many empty rows should be between walls
constexpr auto MAX_SPACE_BETWEEN_WALLS = 7;          // at most how many empty rows should be between walls
constexpr auto MAX_SPACE_BETWEEN_WALLS_PLUS_1 = MAX_SPACE_BETWEEN_WALLS + 1;
constexpr auto DEFAULT_SPACE_BETWEEN_WALLS = MIN_SPACE_BETWEEN_WALLS;
constexpr auto DEAULT_WALL_GAP_SIZE = 3;

constexpr float INIT_PITCH = radians(175.0f);    // [radian] Rotation around Y axis needed to start the game as the MPU6050 needs time for stabilization in an upright position. Specify in degree inside radians().
constexpr float MENU_BACK_ROLL = radians(125.0f);  // [radian] Rotation around X axis needed to restart the game. Specify in degree inside radians().
constexpr float MENU_ENTER_ROLL = radians(-155.0f);
constexpr auto TILT_ANGLE = 165;
constexpr float MENU_NAVIGATION_PITCH = radians(TILT_ANGLE);

constexpr auto HISCORE_ADDRESS = 0;

enum class PlayerMove : int8_t {
    up = -1,
    none = 0,
    down = 1
};

enum class State : uint8_t {
    titleMsg = 0,      // marquee title after boot
    menu,              // selectable games (#TODO: Mazinator game), selectable options: play, demo, hiscore
    changeToPlay,      // initialize play mode
    playMessage,       // flash the following words: rdy, set and go!
    play,              // game is played by the user
    changeToGameOver,  // init game over
    gameOver,          // collision has been detected so display endgame screen
    gameScore,         // display score after game over
    demoMsg,           // flash the word: DEMO
    demo,              // game is played by AI
    hiscore            // all-time highscore is displayed (or top 10 scores could be scrolled)
};

enum class MenuState : int8_t {
    play = 0,
    demo,
    hiscore,
    length
};

enum class MenuPitchState : int8_t {
    center = -1,
    left = (int8_t)LEDMatrixDriver::scrollDirection::scrollUp,
    right = (int8_t)LEDMatrixDriver::scrollDirection::scrollDown
};

// Function prototypes (only the necessary ones)
uint8_t createWall(uint8_t gapSize = 2);
PlayerMove calcPlayerMove();
void movePlayer(PlayerMove playerMove);
void setMenuTextByState(char* menuText, MenuState menuState);

// Global objects and structs
LEDMatrixDriver ledMatrix(BLOCK_COUNT, LEDMATRIX_CS_PIN, LEDMatrixDriver::INVERT_SEGMENT_X | LEDMatrixDriver::INVERT_DISPLAY_X | LEDMatrixDriver::INVERT_Y);
MPU6050Libized motion(2);
YawPitchRoll ypr;

// Global variables
uint32_t currentTime;
uint8_t emptyScrolls = 255;
uint8_t playerX = 2, playerY = 3;
bool playerState = LOW;

uint8_t displayIntensity = LEDMATRIX_DEFAULT_INTENSITY;
uint32_t score = 0;
bool isScoreDisplayed = false;
uint32_t gameOverTime = 0;
uint8_t intensifyCylceCount = 0;
uint8_t playMsgUpdatesCount = 0;
uint8_t currentPlayMessage = 0;
bool isDemoMsgDisplayed = false;
uint32_t demoTriggeredTime = 0;
bool isFirstTextScroll = true;
bool isMenuStepped = false;
bool isHiscoreDisplayed = false;
uint32_t highScoreEnteredTime = 0;
uint32_t lastMenuNavigationTime = 0;

State state;
MenuState menuState = MenuState::play;
MenuPitchState menuPitchState = MenuPitchState::center;

// Global arrays
uint8_t* frameBuffer;
const uint16_t multiplesOfTen[] = { 1000, 100, 10, 1 };








void setup() {
    // init RNG
    randomSeed(generateRandomSeed());
    random();

    Serial.begin(115200);

    // init motion
    if (motion.init(X_ACCELERATION_OFFSET, Y_ACCELERATION_OFFSET, Z_ACCELERATION_OFFSET, X_GYROSCOPE_OFFSET, Y_GYROSCOPE_OFFSET, Z_GYROSCOPE_OFFSET)) {
        state = State::titleMsg;
    }

    // init display
    ledMatrix.setEnabled(true);
    ledMatrix.display();
    ledMatrix.setIntensity(displayIntensity);

    frameBuffer = ledMatrix.getFrameBuffer();

    // Display text containing any alphanumeric characters
    /*
    displayText("1 dS");
    while (true);
    */

    // Chasing dot for testing purposess
    /*
    while (true) {
        for (int y = 0; y < BLOCK_ROW_COUNT; y++) {
            for (int x = 0; x < LEDMATRIX_WIDTH; x++) {
                ledMatrix.setPixel(x, y, HIGH);
                ledMatrix.display();
                delay(25);
                ledMatrix.setPixel(x, y, LOW);
                ledMatrix.display();
                delay(25);
            }
        }    
    }
    */
    
    // Title msg test
    //while (true) {
    //    for (int i = 0; i < strlen("WALL SCROLLER    ") * 8; i++) {
    //        //if (i > 8 * 8) {
    //        //    ledMatrix.scroll(LEDMatrixDriver::scrollDirection::scrollUp);
    //        //    ledMatrix.display();
    //        //} else {
    //            scrollText("WALL SCROLLER    ");
    //        //}
    //        delay(20);
    //    }
    //}

    uint32_t hiscore = 0;
    //EEPROM.put(HISCORE_ADDRESS, hiscore);
    EEPROM.get(HISCORE_ADDRESS, hiscore);
    Serial.print("Hiscore: ");
    Serial.println(hiscore);
}

void loop() {
    currentTime = millis();

    // process data from MPU6050
    if (motion.checkMPUDataAvailable()) {
        ypr = motion.getYawPitchRoll();
    }

    switch (state) {
    case State::titleMsg:
        displayTitle();
        break;
    case State::menu:
        handleMenu();
        break;
    case State::changeToPlay:
        initPlay();
        break;
    case State::playMessage:
        displayPlayMsg();
        break;
    case State::play:
        gameLoop();
        break;
    case State::changeToGameOver:
        initGameOver();
        break;
    case State::gameOver:
        handleGameOver();
        break;
    case State::gameScore:
        displayGameScore();
        break;
    case State::demoMsg:
        displayDemoMsg();
        break;
    case State::demo:
        gameLoop();
        break;
    case State::hiscore:
        displayHiscore();
        break;
    default:
        displayText("BUG!");
        break;
    }
}








// generates random seed according to the von Neumann method
uint32_t generateRandomSeed() {
    uint8_t storedBits = 0;
    uint32_t seed = 0;
    analogRead(A0); analogRead(A0);  // burn first two analog values to stabilize next analogReads

    while (storedBits < sizeof(uint64_t) * 8) {
        int firstAnalogValue = bitRead(analogRead(A0), 0);
        delayMicroseconds(128);
        int secondAnalogValue = bitRead(analogRead(A0), 0);

        if (firstAnalogValue != secondAnalogValue) {
            seed = (seed & ~(1UL << storedBits)) | (firstAnalogValue << storedBits);  // clear current bit and set it to the value of firstAnalogValue
            storedBits++;
        }

        delayMicroseconds(128);
    }

    return seed;
}

uint8_t createWall(uint8_t gapSize) {
    uint8_t wallCol = 0xff;
    uint8_t gapStartPos = random(BLOCK_COL_COUNT - gapSize + 1);

    return wallCol ^ (((1 << gapSize) - 1) << gapStartPos);
}

void displayWall(uint8_t wallCol) {
    ledMatrix.scroll(LEDMatrixDriver::scrollDirection::scrollLeft);
    ledMatrix.setColumn(LEDMATRIX_WIDTH - 1, wallCol);
    updatePlayerPosition(playerX, playerY, playerState);  // player pixel must be set according to playerState after the screen has been scrolled
    ledMatrix.display();
}

void displayScroll() {
    ledMatrix.scroll(LEDMatrixDriver::scrollDirection::scrollLeft);
    updatePlayerPosition(playerX, playerY, playerState);
    ledMatrix.display();
}

void updatePlayerPosition(uint8_t x, uint8_t y, bool enabled) {
    if (detectCollision(x, y)) {
        state = State::changeToGameOver;
        //return;  // sometimes the old position must be retained... handle it with a parameter
    }
    ledMatrix.setPixel(playerX, playerY, LOW);  // turn off old position
    ledMatrix.setPixel(playerX ? playerX - 1 : playerX, playerY, LOW);  // turn off scrolled positon
    playerX = x; playerY = y;
    ledMatrix.setPixel(x, y, enabled);
}

bool detectCollision(uint8_t playerX, uint8_t playerY) {
    return ledMatrix.getPixel(playerX, playerY);
}

PlayerMove calcPlayerMove() {
    // determine closest wall coloumn
    uint8_t wallColPos = 0;
    for (int x = LEDMATRIX_WIDTH - 1; x > playerX; x--) {
        for (int y = 0; y < BLOCK_ROW_COUNT; y++) {
            if (bitRead(frameBuffer[y * BLOCK_COUNT + x / 8], 7 - x % 8)) {
                wallColPos = x;
                break;
            }
        }
    }

    // store gap position in the wall
    uint8_t wallGap = 0;
    for (int y = 0; y < BLOCK_ROW_COUNT; y++) {
        if (bitRead(frameBuffer[y * BLOCK_COUNT + wallColPos / 8], 7 - wallColPos % 8) == 0) {
            wallGap |= (1 << y);
        }
    }

    uint8_t gapBelowPlayerMask = ~(0xFF & (1 << (playerY + 1)) - 1);  // bit mask used to determine if the gap is below the player
    if (bitRead(wallGap, playerY)) {
        return PlayerMove::none;
    } else if (wallGap & gapBelowPlayerMask) {
        return PlayerMove::down;
    } else {
        return PlayerMove::up;
    }
}

void movePlayer(PlayerMove playerMove) {
    if (playerMove != PlayerMove::none)
        updatePlayerPosition(playerX, playerY + (int8_t)playerMove, playerState);
}

void displayScore(uint16_t score) {
    ledMatrix.clear();
    ledMatrix.setIntensity(LEDMATRIX_DEFAULT_INTENSITY);

    for (uint8_t block = 0; block < 4; ++block) {
        int digit = 0;
        while (score >= multiplesOfTen[block]) {
            ++digit;
            score -= multiplesOfTen[block];
        }
        //displayedCodes[block] = NUMBER_CODES_UPDOWN[digit];
        uint64_t mask = 0xFF;
        uint64_t number;
        memcpy_P(&number, &NUMBER_CODES_UPDOWN[digit], sizeof(number));  // NUMBER_CODES_UPDOWN is stored in PROGMEM, memcpy_P is needed to access its values
        for (int x = 24 - block * BLOCK_COL_COUNT; x < 24 - block * BLOCK_COL_COUNT + BLOCK_COL_COUNT; x++) {
            ledMatrix.setColumn(x, (number & mask) >> ((x - (24 - block * BLOCK_COL_COUNT)) * 8));
            mask = mask << 8;
        }
    }
    ledMatrix.display();
}

void resetVariables() {
    score = 0;
    isScoreDisplayed = false;
    emptyScrolls = 255;
    playerState = LOW;
    playMsgUpdatesCount;
    currentPlayMessage = 0;
    isDemoMsgDisplayed = false;
    isFirstTextScroll = true;
    isMenuStepped = false;
    menuPitchState = MenuPitchState::center;
    isHiscoreDisplayed = false;
    highScoreEnteredTime = 0;
    lastMenuNavigationTime = 0;
    ledMatrix.clear();
    ledMatrix.setIntensity(LEDMATRIX_DEFAULT_INTENSITY);
    ledMatrix.display();
}

const void* getCharCode(char character) {
    const void* charCode;
    if (character >= 'A' && character <= 'Z') {
        charCode = &CHAR_CODES_UPPER_UPDOWN[character - 'A'];
    } else if (character >= 'a' && character <= 'z') {
        charCode = &CHAR_CODES_LOWER_UPDOWN[character - 'a'];
    } else if (character >= '0' && character <= '9') {
        charCode = &NUMBER_CODES_UPDOWN[character - '0'];
    } else if (character == '!') {
        charCode = &exclamationMark;
    } else {
        charCode = &space;
    }
    return charCode;
}

void displayText(const char* text) {
    ledMatrix.clear();
    size_t textLength = strlen(text);
    uint64_t letter = 0;
    for (int block = 0; block < (textLength > BLOCK_COUNT ? BLOCK_COUNT : textLength); ++block) {
        uint64_t mask = 0xFF;
        memcpy_P(&letter, getCharCode(text[block]), sizeof(letter));
        for (int x = 24 - block * BLOCK_COL_COUNT; x < 24 - block * BLOCK_COL_COUNT + BLOCK_COL_COUNT; x++) {
            ledMatrix.setColumn(x, (letter & mask) >> ((x - (24 - block * BLOCK_COL_COUNT)) * 8));
            mask = mask << 8;
        }
    }
    ledMatrix.display();
}

// scrolls text from bottom to top
void scrollText(const char* text) {
    static size_t scrolledTextLength;
    static uint16_t scrollCount;
    static uint64_t currScrolledLetter = 0;
    //static int8_t currScrolledBlock = 0;
    static uint64_t scrollMask = 0xFF00000000000000; //0xFF;
    
    if (isFirstTextScroll) {
        ledMatrix.clear();
        scrolledTextLength = strlen(text);
        scrollCount = 0;
        //currScrolledBlock = -1;
        scrollMask = 0xFF00000000000000;  //0xFF;
        isFirstTextScroll = false;
    }

    if (scrollCount % 8 == 0) {
        memcpy_P(&currScrolledLetter, getCharCode(text[(scrollCount / 8) % scrolledTextLength]), sizeof(currScrolledLetter));
        scrollMask = 0xFF00000000000000;  //0xFF;
        //++currScrolledBlock;
    }

    ledMatrix.scroll(LEDMatrixDriver::scrollDirection::scrollRight);

    ledMatrix.setColumn(0, (currScrolledLetter & scrollMask) >> ((7 - (scrollCount % 8)) * 8));
    scrollMask = scrollMask >> 8;
    ++scrollCount;

    ledMatrix.display();
}

void displayTextScroll(const char* text) {
    static uint32_t lastTextScrollTime = 0;
    if (currentTime - lastTextScrollTime > TEXT_SCROLL_INTERVAL) {
        scrollText(text);
        lastTextScrollTime = currentTime;
    }
}

void displayTitle() {
    static const char* title = "WALL SCROLLER     ";
    static uint32_t lastTitleScrollTime = 0;
    if (abs(ypr.pitch) > INIT_PITCH) {
        resetVariables();
        //state = State::playMessage;
        state = State::menu;
    } else {
        displayTextScroll(title);
    }
}

void setMenuTextByState(char* menuText, MenuState menuState) {
    if (menuState == MenuState::play) {
        strcpy(menuText, "PLAY    ");
    } else if (menuState == MenuState::demo) {
        strcpy(menuText, "DEMO    ");
    } else if (menuState == MenuState::hiscore) {
        strcpy(menuText, "HISCORE    ");
    }
}

void handleMenu() {
    static char menuText[16] = "PLAY    ";
    static uint8_t menuScrollCount = 8;

    // #TODO: handle menu back command (relevant if more than one game is implemented)

    // handle menu navigation
    if (menuPitchState == MenuPitchState::center && ypr.pitch > -MENU_NAVIGATION_PITCH && ypr.pitch < -MENU_NAVIGATION_PITCH + radians(20)) {
        // left
        menuPitchState = MenuPitchState::left;
        menuState = (int8_t)menuState - 1 < 0 ? MenuState::hiscore : (MenuState)((int8_t)menuState - 1);
        setMenuTextByState(menuText, menuState);
        isFirstTextScroll = true;
        menuScrollCount = 0;
        lastMenuNavigationTime = currentTime;
    } else if (menuPitchState == MenuPitchState::center && ypr.pitch < MENU_NAVIGATION_PITCH && ypr.pitch > MENU_NAVIGATION_PITCH - radians(20)) {
        // right
        menuPitchState = MenuPitchState::right;
        menuState = (MenuState)(((int8_t)menuState + 1) % (int8_t)MenuState::length);
        setMenuTextByState(menuText, menuState);
        isFirstTextScroll = true;
        menuScrollCount = 0;
        lastMenuNavigationTime = currentTime;
    } else if ((menuPitchState == MenuPitchState::left || menuPitchState == MenuPitchState::right) && (ypr.pitch < radians(-170) && ypr.pitch > radians(-180) || ypr.pitch > radians(170) && ypr.pitch < radians(180))) {
        // center
        menuPitchState = MenuPitchState::center;
    }

    // #TODO: first 8 scrolls should scroll out text in menuPitchState direction
    if (menuScrollCount < 8) {
        ++menuScrollCount;
        ledMatrix.scroll((LEDMatrixDriver::scrollDirection)menuPitchState);
        ledMatrix.display();
        delay(TEXT_SCROLL_INTERVAL);  // #TODO: should not use delay()!
    } else {
        displayTextScroll(menuText);
    }

    // handle menu enter command
    if (ypr.roll > MENU_ENTER_ROLL&& ypr.roll < MENU_ENTER_ROLL + radians(20.0f)) {
        if (menuState == MenuState::play) {
            state = State::changeToPlay;
        } else if (menuState == MenuState::demo) {
            state = State::demoMsg;
        } else if (menuState == MenuState::hiscore) {
            state = State::hiscore;
            highScoreEnteredTime = currentTime;
        }
    }
}

// uses globals: playMsgUpdatesCount, currentPlayMessage
void displayPlayMsg() {
    static uint32_t lastPlayMsgUpdateTime;
    static const char* playMessages[] = { "RDY", "SET", "GO!" };
    static const uint8_t playMessagesSize = sizeof(playMessages) / sizeof(playMessages[0]);
    if (currentTime - lastPlayMsgUpdateTime > PLAY_MSG_UPDATE_INTERVAL) {
        uint8_t playMsgUpdatesMod5 = playMsgUpdatesCount % 5;
        if (currentPlayMessage < playMessagesSize || playMsgUpdatesMod5 <= 3) {
            if (playMsgUpdatesMod5 == 0) {
                displayText(playMessages[currentPlayMessage++]);
            } else if (playMsgUpdatesMod5 == 1) {
                ledMatrix.setIntensity(LEDMATRIX_MAX_INTENSITY);
            } else if (playMsgUpdatesMod5 == 3) {
                ledMatrix.setIntensity(LEDMATRIX_DEFAULT_INTENSITY);
            }
        } else {
            ledMatrix.clear();
            state = State::play;
        }
        ++playMsgUpdatesCount;
        lastPlayMsgUpdateTime = currentTime;
    }
}

void displayDemoMsg() {
    static const char* demoText = " DEMO      ";

    displayTextScroll(demoText);
    if (currentTime - demoTriggeredTime > DEMO_MSG_DISPLAY_INTERVAL) {
        resetVariables();
        state = State::demo;
    }

    // simple text display
    /*if (!isDemoMsgDisplayed) {
        resetVariables();
        displayText("DEMO");
        isDemoMsgDisplayed = true;
    } else if (currentTime - demoTriggeredTime > DEMO_MSG_DISPLAY_INTERVAL) {
        ledMatrix.clear();
        state = State::demo;
    }*/
}

void initPlay() {
    if (abs(ypr.pitch) > INIT_PITCH) {
        resetVariables();
        state = State::playMessage;
    }
}

void gameLoop() {
    // Scroll the screen
    static uint32_t lastScrollTime, lastPlayerBlinkTime, lastPlayerPosUpdateTime, scrollInterval = DEFAULT_SCROLL_INTERVAL;
    static uint8_t wallCol, wallsLeft, currentSpaceLeftBetweenWalls = DEFAULT_SPACE_BETWEEN_WALLS;
    if (currentTime - lastScrollTime > scrollInterval) {
        // create wall if enough empty scrolls have been made
        if (emptyScrolls >= currentSpaceLeftBetweenWalls) {
            wallCol = createWall(DEAULT_WALL_GAP_SIZE);
            wallsLeft = random(MIN_WALL_THICKNESS, MAX_WALL_THICKNESS + 1);
            emptyScrolls = 0;
            int speedCorrection = map(scrollInterval, DEFAULT_SCROLL_INTERVAL, MIN_SCROLL_INTERVAL, 0, 4);
            currentSpaceLeftBetweenWalls = random(MIN_SPACE_BETWEEN_WALLS + speedCorrection, MAX_SPACE_BETWEEN_WALLS_PLUS_1 + speedCorrection);
        }

        PlayerMove playerMove = PlayerMove::none;
        if (state == State::demo) {
            // AI player move calculation
            playerMove = calcPlayerMove();
        }

        // display walls if it's appropriate otherwise just scroll the screen
        if (wallsLeft) {
            displayWall(wallCol);
            --wallsLeft;
        } else {
            displayScroll();
            ++emptyScrolls;
        }

        if (state == State::demo) {
            // AI player movement
            movePlayer(playerMove);
            ledMatrix.display();
        }

        ++score;
        scrollInterval = map(constrain(score, 0, 600), 0, 600, DEFAULT_SCROLL_INTERVAL, MIN_SCROLL_INTERVAL);

        lastScrollTime = currentTime;
    }

    // Update player position
    if (currentTime - lastPlayerPosUpdateTime > PLAYER_POS_UPDATE_INTERVAL) {
        if (state == State::play) {
            int angleY = round(degrees(ypr.pitch));
            if (angleY < 0) {
                angleY = map(constrain(angleY, -180, -TILT_ANGLE), -180, -TILT_ANGLE, TILT_ANGLE, 140);
            }
            else
                angleY = map(constrain(angleY, TILT_ANGLE, 180), TILT_ANGLE, 180, 180, TILT_ANGLE);

            uint8_t newPlayerY = map(angleY, 140, 180, 0, 7);

            if (newPlayerY != playerY) {
                updatePlayerPosition(playerX, newPlayerY, playerState);
                ledMatrix.display();
            }
        } else if (state == State::demo && ypr.roll < MENU_BACK_ROLL && ypr.roll > MENU_BACK_ROLL - radians(20.0f)) {
            // Go back to Menu
            state = State::menu;
            //state = State::changeToPlay;
        }

        lastPlayerPosUpdateTime = currentTime;
    }

    // Blink player pixel - turn off
    if ((currentTime - lastPlayerBlinkTime > PLAYER_SHOW_INTERVAL) && playerState) {
        ledMatrix.setPixel(playerX, playerY, LOW);
        ledMatrix.display();
        playerState = LOW;

        lastPlayerBlinkTime = currentTime;
    }

    // Blink player pixel - turn on
    if ((currentTime - lastPlayerBlinkTime > PLAYER_HIDE_INTERVAL) && !playerState) {
        ledMatrix.setPixel(playerX, playerY, HIGH);
        ledMatrix.display();
        playerState = HIGH;

        lastPlayerBlinkTime = currentTime;
    }
}

void initGameOver() {
    intensifyCylceCount = 0;
    state = State::gameOver;
}

void handleGameOver() {
    static uint32_t lastEndgamePlayerBlinkTime, lastEndgameIntensifyTime;
    // Blink player pixel slowly
    if (currentTime - lastEndgamePlayerBlinkTime > ENDGAME_PLAYER_BLINK_INTERVAL) {
        ledMatrix.setPixel(playerX, playerY, playerState);
        ledMatrix.display();
        playerState = !playerState;

        lastEndgamePlayerBlinkTime = currentTime;
    }

    // Intensify game over
    if (currentTime - lastEndgameIntensifyTime > ENDGAME_INSTENSIFY_INTERVAL) {
        displayIntensity = (displayIntensity + 1) % 32;
        ledMatrix.setIntensity(displayIntensity < 16 ? displayIntensity : 31 - displayIntensity);

        // do ENDGAME_INSTENSIFY_CYCLES_COUNT intensify cycles then display score
        if (displayIntensity == 0 && ++intensifyCylceCount == ENDGAME_INSTENSIFY_CYCLES_COUNT) {
            state = State::gameScore;
            gameOverTime = currentTime;
        }

        lastEndgameIntensifyTime = currentTime;
    }
}

void displayGameScore() {
    if (!isScoreDisplayed) {
        displayScore(score);
        isScoreDisplayed = true;

        uint32_t hiscore = 0;
        EEPROM.get(HISCORE_ADDRESS, hiscore);
        if (score > hiscore) {
            EEPROM.put(HISCORE_ADDRESS, score);  // save hiscore
        }
    }

    // change to demo mode (AI control) in case there is no user interaction for a long time
    if (currentTime - gameOverTime > ENDGAME_AFK_INTERVAL) {
        state = State::demoMsg;
        demoTriggeredTime = currentTime;
    }

    // change to play mode if MENU_ENTER_ROLL angle is reached
    /*
    if (ypr.roll > MENU_ENTER_ROLL && ypr.roll < MENU_ENTER_ROLL + radians(20.0f)) {
        state = State::changeToPlay;
    }
    */
    if (ypr.roll < MENU_BACK_ROLL && ypr.roll > MENU_BACK_ROLL - radians(20.0f)) {
        // Go back to Menu
        state = State::menu;
        resetVariables();
    }
}

void displayHiscore() {
    if (!isHiscoreDisplayed) {
        uint32_t hiscore = 0;
        EEPROM.get(HISCORE_ADDRESS, hiscore);
        displayScore(hiscore);
        isHiscoreDisplayed = true;
    }

    // change to demo mode (AI control) in case there is no user interaction for a long time
    if (currentTime - highScoreEnteredTime > ENDGAME_AFK_INTERVAL) {
        state = State::demoMsg;
        demoTriggeredTime = currentTime;
    }

    // go back to Menu when the device is tilted towards the user
    if (ypr.roll < MENU_BACK_ROLL && ypr.roll > MENU_BACK_ROLL - radians(20.0f)) {
        // Go back to Menu
        state = State::menu;
        resetVariables();
    }
}
