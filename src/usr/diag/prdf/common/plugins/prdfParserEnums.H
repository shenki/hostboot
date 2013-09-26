/* IBM_PROLOG_BEGIN_TAG                                                   */
/* This is an automatically generated prolog.                             */
/*                                                                        */
/* $Source: src/usr/diag/prdf/common/plugins/prdfParserEnums.H $          */
/*                                                                        */
/* IBM CONFIDENTIAL                                                       */
/*                                                                        */
/* COPYRIGHT International Business Machines Corp. 2013                   */
/*                                                                        */
/* p1                                                                     */
/*                                                                        */
/* Object Code Only (OCO) source materials                                */
/* Licensed Internal Code Source Materials                                */
/* IBM HostBoot Licensed Internal Code                                    */
/*                                                                        */
/* The source code for this program is not published or otherwise         */
/* divested of its trade secrets, irrespective of what has been           */
/* deposited with the U.S. Copyright Office.                              */
/*                                                                        */
/* Origin: 30                                                             */
/*                                                                        */
/* IBM_PROLOG_END_TAG                                                     */

/** @file  prdfParserEnums.H
 *  @brief Common enums used for building and parsing capture data.
 */

#ifndef __prdfParserEnums_H
#define __prdfParserEnums_H

#include <prdfCenConst.H>

namespace PRDF
{

namespace PARSER
{
    enum
    {
        // i_parser.PrintString() has two parts: header and data. These are the
        // maximum string lengths for one line in the error log. Note that the
        // header has a hard stop of one line, however, the data can wrap onto
        // multiple lines (not sure of the line limit).
        HEADER_SIZE  = 25,
        DATA_SIZE    = 50,
    };

} // namespace PARSER

namespace UE_TABLE
{
    enum
    {
        MAX_ENTRIES = 16, ///< Maximum number of entries allow in table.
        ENTRY_SIZE  =  7, ///< Number of bytes per entry.

        MAX_SIZE = MAX_ENTRIES * ENTRY_SIZE, ///< Maximum table size.
    };

    /**
     * @brief Represents the several different types of UEs stored in the table.
     */
    enum Type
    {
        SCRUB_MPE = 1, ///< Chip mark placed by scrub
        FETCH_MPE,     ///< Chip mark placed by fetch
        SCRUB_UE,      ///< Scrub UE
        FETCH_UE,      ///< Fetch UE
    };

} // namespace UE_TABLE

namespace DQ_BITMAP
{
    enum
    {
        BITMAP_SIZE = 10, // 80-bit bitmap
        ENTRY_SIZE  = sizeof(uint8_t) + PORT_SLCT_PER_MBA * BITMAP_SIZE,
    };

} // namespace DQ_BITMAP

} // end namespace PRDF

#endif // __prdfParserEnums_H
