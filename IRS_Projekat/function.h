/**
 * @file function.h
 * @brief Declaration of function used to write to 7seg display
 *
 * @date 2017
 * @author Strahinja Jankovic (jankovics@etf.bg.ac.rs)
 */

#ifndef FUNCTION_H_
#define FUNCTION_H_

/**
 * @brief Function used to write to 7seg display
 * @param digit - value 0-9 to be displayed
 *
 * Function writes data a-g on PORT6.
 * It is assumed that appropriate 7seg display is enabled.
 */
extern void WriteLed(int digit);

#endif /* FUNCTION_H_ */
