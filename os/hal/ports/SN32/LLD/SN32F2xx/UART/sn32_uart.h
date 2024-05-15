/*
    Copyright (C) 2023 Dimitris Mantzouranis

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef SN32_UART_H
#    define SN32_UART_H

typedef struct {
    union {
        union {
            __IM uint32_t RB;

            struct {
                __IM uint32_t RB : 8;
            } RB_b;
        };

        union {
            __OM uint32_t TH;

            struct {
                __OM uint32_t TH : 8;
            } TH_b;
        };

        union {
            __IOM uint32_t DLL;

            struct {
                __IOM uint32_t DLL : 8;
            } DLL_b;
        };
    };

    union {
        union {
            __IOM uint32_t DLM;

            struct {
                __IOM uint32_t DLM : 8;
            } DLM_b;
        };

        union {
            __IOM uint32_t IE;

            struct {
                __IOM uint32_t RDAIE : 1;
                __IOM uint32_t THREIE : 1;
                __IOM uint32_t RLSIE : 1;
                __IM           uint32_t : 1;
                __IOM uint32_t TEMTIE : 1;
                __IM           uint32_t : 3;
                __IOM uint32_t ABEOIE : 1;
                __IOM uint32_t ABTOIE : 1;
            } IE_b;
        };
    };

    union {
        union {
            __IM uint32_t II;

            struct {
                __IM uint32_t INTSTATUS : 1;
                __IM uint32_t INTID : 3;
                __IM          uint32_t : 2;
                __IM uint32_t FIFOEN : 2;
                __IM uint32_t ABEOIF : 1;
                __IM uint32_t ABTOIF : 1;
            } II_b;
        };

        union {
            __OM uint32_t FIFOCTRL;

            struct {
                __OM uint32_t FIFOEN : 1;
                __IM          uint32_t : 5;
                __OM uint32_t RXTL : 2;
            } FIFOCTRL_b;
        };
    };

    union {
        __IOM uint32_t LC;

        struct {
            __IOM uint32_t WLS : 2;
            __IOM uint32_t SBS : 1;
            __IOM uint32_t PE : 1;
            __IOM uint32_t PS : 2;
            __IOM uint32_t BC : 1;
            __IOM uint32_t DLAB : 1;
        } LC_b;
    };
    __IM uint32_t RESERVED;

    union {
        __IM uint32_t LS;

        struct {
            __IM uint32_t RDR : 1;
            __IM uint32_t OE : 1;
            __IM uint32_t PE : 1;
            __IM uint32_t FE : 1;
            __IM uint32_t BI : 1;
            __IM uint32_t THRE : 1;
            __IM uint32_t TEMT : 1;
            __IM uint32_t RXFE : 1;
        } LS_b;
    };
    __IM uint32_t RESERVED1;

    union {
        __IOM uint32_t SP;

        struct {
            __IOM uint32_t PAD : 8;
        } SP_b;
    };

    union {
        __IOM uint32_t ABCTRL;

        struct {
            __IOM uint32_t START : 1;
            __IOM uint32_t MODE : 1;
            __IOM uint32_t AUTORESTART : 1;
            __IM           uint32_t : 5;
            __OM uint32_t  ABEOIFC : 1;
            __OM uint32_t  ABTOIFC : 1;
        } ABCTRL_b;
    };
    __IM uint32_t RESERVED2;

    union {
        __IOM uint32_t FD;

        struct {
            __IOM uint32_t DIVADDVAL : 4;
            __IOM uint32_t MULVAL : 4;
            __IOM uint32_t OVER8 : 1;
        } FD_b;
    };
    __IM uint32_t RESERVED3;

    union {
        __IOM uint32_t CTRL;

        struct {
            __IOM uint32_t UARTEN : 1;
            __IOM uint32_t MODE : 3;
            __IM           uint32_t : 2;
            __IOM uint32_t RXEN : 1;
            __IOM uint32_t TXEN : 1;
        } CTRL_b;
    };

    union {
        __IOM uint32_t HDEN;

        struct {
            __IOM uint32_t HDEN : 1;
        } HDEN_b;
    };
} sn32_uart_t;

#endif /* SN32_UART_H */

/** @} */
