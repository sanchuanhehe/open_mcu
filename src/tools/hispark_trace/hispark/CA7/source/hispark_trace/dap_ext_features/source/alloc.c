/**
  * @copyright Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2012-2023. All rights reserved.
  * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
  * following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
  * disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
  * following disclaimer in the documentation and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
  * products derived from this software without specific prior written permission.
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
  * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  * @file    alloc.c
  * @author  MCU Driver Team
  * @brief   Memory Alloc Process
  */
#include <string.h>
#include <stdbool.h>
#include <stddef.h>
#include "debug.h"
#include "securec.h"

#pragma pack(1)
typedef struct MemBlock {                 /* Memory Mangement Block Struct */
    void                *memPtr;          /* Managed Address of memory block */
    struct MemBlock     *nextPtr;         /* Next memory block */
    uint32_t             memSize;         /* Memory Size */
    bool                 memUsed;         /* Used or Free */
} MemBlock;
#pragma pack()
extern uint32_t __HEAP_START__;  /* defined by linker script */
extern uint32_t __HEAP_END__;    /* defined by linker script */

#define MEM_SIZE  ((size_t)((void *)&__HEAP_END__ - (void *)&__HEAP_START__))
static const void *MEM_START = (void *)&__HEAP_START__;
static const void *MEM_END = (void *)&__HEAP_END__;
static const uint32_t BLOCK_SIZE = sizeof(MemBlock);
static MemBlock *g_hdrNode = NULL;
static bool g_memInited = false;      /* Memory Management Inited Flag */

/*
 * Init Memory Management
 */
static void Init(void)
{
    uint32_t *heap = NULL;
    MemBlock *node = NULL;
    errno_t ret;

    /* Init hdr node, don't change g_hdrNode anymore */
    g_hdrNode = (MemBlock *)(MEM_END - BLOCK_SIZE);
    
    heap = (uint32_t *)MEM_START;
    ret = memset_s((void*)heap, MEM_SIZE, 0, MEM_SIZE);
    if (ret != EOK) {
        return;
    }

    /* Init first node */
    node = g_hdrNode;
    node->memPtr  = (void *)MEM_START;
    node->nextPtr = g_hdrNode;
    node->memSize = (uint32_t)(MEM_SIZE - BLOCK_SIZE);
    node->memUsed  = false;

    g_memInited = true; /* Init done */
}

/*
 * Find Suit Node In Free Nodes
 */
static MemBlock *FindSuitNode(uint32_t nbytes)
{
    uint32_t suitSize = (uint32_t)-1;
    MemBlock *hdrNode = g_hdrNode; /* always find begin with hdr node */
    MemBlock *tmpNode = g_hdrNode;
    MemBlock *suitNode = NULL;

    while (true) {
        if (!tmpNode->memUsed) { /* Only check unused node */
            /* find node with suit size */
            if ((nbytes <= tmpNode->memSize) && (tmpNode->memSize < suitSize)) {
                suitNode = tmpNode;             /* update suit node and size */
                suitSize = suitNode->memSize;
            }
        }
        tmpNode = tmpNode->nextPtr;
        if (tmpNode == hdrNode) {
            break;  /* already check all nodes, break */
        }
    }
    return suitNode;
}

/*
 * Find Suit Node In Free Nodes
 */
static void *AllocNode(MemBlock *suitNode, uint32_t nbytes)
{
    /* Create a new node from suit nodes */
    MemBlock *tmpNode = (MemBlock *)suitNode->memPtr;
    tmpNode = (MemBlock *)((unsigned char *)tmpNode + nbytes);
    tmpNode->memPtr  = suitNode->memPtr;
    tmpNode->nextPtr = suitNode->nextPtr;
    tmpNode->memSize = nbytes;
    tmpNode->memUsed  = true;

    /* update suit node Pointer to the next unused memory */
    suitNode->memPtr   = (MemBlock *)((unsigned char *)tmpNode + BLOCK_SIZE);
    suitNode->nextPtr  =  tmpNode;
    suitNode->memSize -= (nbytes + BLOCK_SIZE);
    suitNode->memUsed   = false;

    return tmpNode->memPtr;  /* return the new node */
}

/*
 * Free Node, merge two unused node
 */
static void MergeFreeNode(MemBlock *freeNode, MemBlock *mergeNode)
{
    /* update free node */
    freeNode->memPtr   = mergeNode->memPtr;
    freeNode->nextPtr  = mergeNode->nextPtr;
    freeNode->memSize += mergeNode->memSize + BLOCK_SIZE;
    freeNode->memUsed  = false;
}

/*
 * Find Node by address
 */
static void *FindNode(const void *ptr)
{
    MemBlock *hdrNode = g_hdrNode;
    MemBlock *tmpNode = g_hdrNode;

    while (true) {
        if (tmpNode->memPtr == ptr) {
            if (tmpNode->memUsed) {
                /* find the node and is used, make it unused(free) */
                tmpNode->memUsed = false;
                break;
            } else {
                /* find the node and is unused, it's free already, return */
                DBG_PRINTF("ptr:0x%08x already free\r\n", ptr);
                return NULL;
            }
        }
        /* try next node */
        tmpNode = tmpNode->nextPtr;
        if (tmpNode == hdrNode) {
            /* ERROR: we have check all nodes and can't find the node */
            DBG_PRINTF("%s %d, can't find the node, free fail\r\n", __func__, __LINE__);
            return NULL;
        }
    }
    return tmpNode;
}

/*
 * Combine all consecutive Free nodes.
 */
static void MergeAllFreeNodes(void)
{
    MemBlock *hdrNode = g_hdrNode;
    MemBlock *tmpNode = g_hdrNode;
    MemBlock *nextNode = NULL;

    while (true) {
        nextNode = tmpNode->nextPtr;
        if (nextNode == hdrNode) {
            break;  /* only one node, don't need merge */
        }
        /* If two nodes are adjacent and both nodes are free nodes,
           combine the two nodes. */
        if (!tmpNode->memUsed && !nextNode->memUsed) {
            MergeFreeNode(tmpNode, nextNode);
            hdrNode = g_hdrNode;
            tmpNode = g_hdrNode;
            continue;
        }
        tmpNode = nextNode; /* Try next node */
    }
}

/*
 * Combine all consecutive Free nodes.
 */
void *MemAlloc(uint32_t nbytes)
{
    MemBlock *suitNode = NULL;

    if (nbytes == 0) { /* don't alloc if size = 0 */
        DBG_PRINTF("Invalid Paramter");
        return NULL;
    }
    if (!g_memInited) { /* if memory isn't init, do init */
        Init();
    }
    /* find the suit node */
    suitNode = FindSuitNode(nbytes);
    if (suitNode == NULL) {
        return NULL; /* can't find a suit node, return null */
    }
    if ((nbytes <= suitNode->memSize) && ((nbytes + BLOCK_SIZE) >= suitNode->memSize)) {
        suitNode->memUsed = true; /* find a suit and unused node, used it */
        return suitNode->memPtr;
    } else if (suitNode->memSize >= (nbytes + BLOCK_SIZE)) {
        /* don't find a suit and unused node, creat it */
        return AllocNode(suitNode, nbytes);
    } else {
        DBG_PRINTF("%s,size err!\r\n", __func__);
    }
    return NULL;
}

/*
 * Free Node by addr
 */
void MemFree(void *ptr)
{
    if ((ptr == NULL) || (!g_memInited)) {
        return;
    }
    /* find the node and do combine process */
    if (FindNode(ptr) != NULL) {
        MergeAllFreeNodes();
    }
}