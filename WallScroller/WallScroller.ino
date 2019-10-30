/*
 Name:		WallScroller.ino
 Created:	10/17/2019 10:09:56 AM
 Author:	Benjamin Balogh
*/

// Includes
#include <LEDMatrixDriver.hpp>

#include "LEDMatrixData.hpp"
#include "MPU6050Libized.h"

// Defines
constexpr auto LEDMATRIX_CS_PIN = 9;  // Chip Select pin of LED matrix PCB

constexpr auto BLOCK_COUNT = 4;      // number of 8x8 pixel matrices;
constexpr auto BLOCK_ROW_COUNT = 8;  // number of rows per block
constexpr auto BLOCK_COL_COUNT = 8;  // number of coloumns per block
constexpr auto LEDMATRIX_WIDTH = BLOCK_COUNT * BLOCK_COL_COUNT;  // pixels in the whole LED matrix

constexpr auto DEFAULT_SCROLL_INTERVAL = 150;  // ms between display scrolls
constexpr auto PLAYER_SHOW_INTERVAL = 60;      // [ms] player pixel on state duration
constexpr auto PLAYER_HIDE_INTERVAL = 30;      // [ms] player pixel off state duration

constexpr auto PLAYER_POS_UPDATE_HZ = 60;    // [Hz] player position update frequency
constexpr auto ENDGAME_INSTENSIFY_HZ = 25;   // [Hz] display intensity change frequency after game over
constexpr auto ENDGAME_PLAYER_BLINK_HZ = 5;  // [Hz] player pixel state change frequency after game over

constexpr auto PLAYER_POS_UPDATE_INTERVAL = 1000 / PLAYER_POS_UPDATE_HZ;        // ms between player position updates
constexpr auto ENDGAME_INSTENSIFY_INTERVAL = 1000 / ENDGAME_INSTENSIFY_HZ;      // ms between display intensity changes after game over
constexpr auto ENDGAME_PLAYER_BLINK_INTERVAL = 1000 / ENDGAME_PLAYER_BLINK_HZ;  // ms between player pixel state changes after game over
constexpr auto ENDGAME_AFK_INTERVAL = 30 * 1000;                                // ms after AI control is automatically turned on when in endgame

constexpr auto MAX_WALL_THICKNESS = 5;               // how thick walls could be
constexpr auto DEFAULT_DISPLAY_INTENSITY = 3;        // default display intensity [0-15]

enum class PlayerMove : int8_t {
    up = -1,
    none = 0,
    down = 1
};

enum class State : uint8_t {
    titleMsg = 0,  // marquee title after boot
    //menu,        // selectable games (#TODO: Mazinator game), selectable options: play, demo, best
    playMsg,       // flash the following words: rdy, set and go!
    play,          // game is played by the user
    gameOver,      // collision has been detected so display the score
    demoMsg,       // flash the word: DEMO
    demo,          // game is played by AI
    //best         // all-time highscore is displayed (or top 10 scores could be scrolled)
};

// Function prototypes (only the necessary ones)
uint8_t createWall(uint8_t gapSize = 2);
PlayerMove calcPlayerMove();
void movePlayer(PlayerMove playerMove);

// Global objects and structs
LEDMatrixDriver ledMatrix(BLOCK_COUNT, LEDMATRIX_CS_PIN, LEDMatrixDriver::INVERT_SEGMENT_X | LEDMatrixDriver::INVERT_DISPLAY_X | LEDMatrixDriver::INVERT_Y);
MPU6050Libized motion(2);
YawPitchRoll ypr;

// Global variables
uint32_t currentTime;
uint32_t lastScrollTime, lastPlayerBlinkTime, lastPlayerPosUpdateTime, lastEndgamePlayerBlinkTime, lastEndgameIntensifyTime;
uint8_t wallCol, emptyScrolls = 255, wallsLeft;
uint8_t playerX = 0, playerY = 3;
bool playerState = LOW; // might be replaced with getPixel()... but it would be slower
bool isCollision = false;
bool isMPUConnected = false;
bool isAIRequested = false;

uint8_t displayIntensity = DEFAULT_DISPLAY_INTENSITY;
uint8_t emptyScrollsGoal = 6;
uint16_t score = 0;
uint16_t scrollInterval = DEFAULT_SCROLL_INTERVAL;
State state;

// Global arrays
uint8_t* frameBuffer;
const uint16_t multiplesOfTen[] = { 1000, 100, 10, 1 };

void setup() {
    // init RNG
    randomSeed(generateRandomSeed());
    random();

    Serial.begin(115200);

    // init motion
    if (motion.init(-2841, -4453, 783, 6, 23, 7))
        isMPUConnected = true;

    // init display
    ledMatrix.setEnabled(true);
    ledMatrix.setIntensity(displayIntensity);
    frameBuffer = ledMatrix.getFrameBuffer();

    // set all ledMatrix blocks to the same digit
 /*   for (int block = 0; block < 4; ++block) {
        uint64_t mask = 0xFF;
        for (int x = 24 - block * BLOCK_COL_COUNT; x < 24 - block * BLOCK_COL_COUNT + BLOCK_COL_COUNT; x++) {
            uint64_t letter = 0;
            memcpy_P(&letter, &DEMO_CODES[block], sizeof(letter));
            ledMatrix.setColumn(x, (letter & mask) >> ((x - (24 - block * BLOCK_COL_COUNT)) * 8));
            mask = mask << 8;
        }
    }
    ledMatrix.display();*/
}

void loop() {
    currentTime = millis();

    // #TODO: change boolean logic to state machine pattern

    if (isCollision) {
        // TODO: pause display scroll, blink player position (where the collision happened) slowly for a couple of times then print score. Shake for a new game -> big acceleration along 2 axes (Y and Z).
        bool shouldDisplayScore = false, isScoreDisplayed = false, shouldRestart = false;
        uint8_t intensifyCylceCount = 0;
        uint32_t gameOverTime = currentTime;  // stores time when game over occured
        while (!shouldRestart) {
            currentTime = millis();

            // process data from MPU6050
            if (motion.checkMPUDataAvailable()) {
                ypr = motion.getYawPitchRoll();
            }

            if (currentTime - lastPlayerPosUpdateTime > PLAYER_POS_UPDATE_INTERVAL&& ypr.roll * 180 / M_PI < -40.0f) {
                shouldRestart = true;
            }

            // change to AI control in case there is no user interaction for a long time
            if (currentTime - gameOverTime > ENDGAME_AFK_INTERVAL) {
                isAIRequested = true;  // triggers AI control
                shouldRestart = true;
                gameOverTime = currentTime;
            }

            if (!shouldDisplayScore) {
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

                    // do 3 intensify cycles then display score
                    if (displayIntensity == 0 && ++intensifyCylceCount == 3) {
                        shouldDisplayScore = true;
                    }

                    lastEndgameIntensifyTime = currentTime;
                }
            } else if (!isScoreDisplayed) {
                // display the score once
                displayScore(score);
                isScoreDisplayed = true;
            }
        }

        // Get ready for another run...
        // ...reset necessary variables to their default values
        isCollision = false;
        resetVariables();
    }

    // process data from MPU6050
    if (motion.checkMPUDataAvailable()) {
        ypr = motion.getYawPitchRoll();
    }

    // Scroll the screen
    if (currentTime - lastScrollTime > scrollInterval) {
        // create wall if enough empty scrolls have been made
        if (emptyScrolls >= emptyScrollsGoal) {
            wallCol = createWall(2);
            wallsLeft = random(1, MAX_WALL_THICKNESS + 1);
            emptyScrolls = 0;
            emptyScrollsGoal = random(5, 8);
        }

        PlayerMove playerMove = PlayerMove::none;
        if (!isMPUConnected || isAIRequested) {
            // AI player move calculation
            playerMove = calcPlayerMove();
        }

        // display walls if it's appropriate otherwise just scroll the screen
        if (wallsLeft) {
            displayWall(wallCol);
            --wallsLeft;
        } else {
            displayScroll();
        }

        if (!isMPUConnected || isAIRequested) {
            // AI player movement
            movePlayer(playerMove);
            ledMatrix.display();
        }

        ++score;
        scrollInterval = map(constrain(score, 0, 600), 0, 600, 200, 40);

        lastScrollTime = currentTime;
    }

    // Update player position
    if (currentTime - lastPlayerPosUpdateTime > PLAYER_POS_UPDATE_INTERVAL) {
        if (isMPUConnected && !isAIRequested) {
            int angleY = round(ypr.pitch * 180 / M_PI);
            uint8_t newPlayerY = map(constrain(angleY, -15, 15), -15, 15, 0, 7);

            if (newPlayerY != playerY) {
                updatePlayerPosition(playerX, newPlayerY, playerState);
                ledMatrix.display();
            }
        } else if (isAIRequested && ypr.roll * 180 / M_PI < -40.0f) {
            // restart with MPU control
            isAIRequested = false;
            resetVariables();
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

// generates random seed according to the von Neumann method
uint32_t generateRandomSeed() {
    uint8_t storedBits = 0;
    uint32_t seed = 0;
    analogRead(A0); analogRead(A0);

    while (storedBits < 32) {
        int a1 = analogRead(A0) & 0x1;
        delayMicroseconds(128);
        int a2 = analogRead(A0) & 0x1;

        if (a1 != a2) {
            seed = (seed & ~(1UL << storedBits)) | (a1 << storedBits);  // clear current bit and set it to the value of a1
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
    ++emptyScrolls;
}

void updatePlayerPosition(uint8_t x, uint8_t y, bool state) {
    if (detectCollision(x, y)) {
        isCollision = true;
        //return;
    }
    ledMatrix.setPixel(playerX, playerY, LOW);  // turn off old position
    playerX = x; playerY = y;
    ledMatrix.setPixel(x, y, state);
}

bool detectCollision(uint8_t playerX, uint8_t playerY) {
    return ledMatrix.getPixel(playerX, playerY);
}

PlayerMove calcPlayerMove() {
    // determine closest wall coloumn
    uint8_t wallColPos = 0;
    for (int x = LEDMATRIX_WIDTH - 1; x >= 1; x--) {
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
    ledMatrix.setIntensity(DEFAULT_DISPLAY_INTENSITY);

    for (uint8_t block = 0; block < 4; ++block) {
        int digit = 0;
        while (score >= multiplesOfTen[block]) {
            ++digit;
            score -= multiplesOfTen[block];
        }
        //displayedCodes[block] = NUMBER_CODES_UPDOWN[digit];
        Serial.print(digit);
        uint64_t mask = 0xFF;
        uint64_t number;
        memcpy_P(&number, &NUMBER_CODES_UPDOWN[digit], sizeof(number));  // NUMBER_CODES_UPDOWN is stored in PROGMEM, memcpy_P is needed to access its values
        for (int x = 24 - block * BLOCK_COL_COUNT; x < 24 - block * BLOCK_COL_COUNT + BLOCK_COL_COUNT; x++) {
            ledMatrix.setColumn(x, (number & mask) >> ((x - (24 - block * BLOCK_COL_COUNT)) * 8));
            mask = mask << 8;
        }
    }
    Serial.println();
    ledMatrix.display();
}

void resetVariables() {
    score = 0;
    emptyScrolls = 255;
    playerState = LOW;
    ledMatrix.clear();
    ledMatrix.setIntensity(DEFAULT_DISPLAY_INTENSITY);
    ledMatrix.display();
}