Memory Utility Libraries on Spritzer for Media & Sensor features.{#spritzer_memutil}
============================
[TOC]

# General # {#spritzer_memutil_general}

We provided Memory Utility Libraries on Spritzer for media & sensor features.
These libraries are composed of "Memory Manager", Message Library, and Simple FIFO.

- "Memory Manager" is the fixed-size memory pools that manages implicit acquisition and release.
- "Message Library" is the inter-task library that sends and receives "Memory Handle" made from "Memory Manager".
- "Simple FIFO"  is the basic FIFO used for passing data between an application and a framework.


---------------------

# Memory Manager (Intelligent Fix Pool) # {#spritzer_memory_manager}

  ## General # {#spritzer_memory_manager_general}

  "Memory Manager" is the fixed-size memory pools that manages implicit acquisition and release.<br>
  It has "Memory Layout" for each applications can be changed dynamically.<br>
  The necessary areas is acquired by "Memory Handle" from the pool and guaranteed until "Memory Handle" is discarded.<br>
  Even if multiple users asynchronously use "Memory Handle", that area is guaranteed and will be released when it is no longer referenced from all.<br>

  ## APIs # {#spritzer_memory_manager_api}
  - @ref memutils_memory_manager

  ## Configurations # {#spritzer_memory_manager_config}

  "Memory Manager" can be layout areas of memory pools by users configuration and their tools.

  ### How to the use tool # {#spritzer_memory_manage_config_how_to}

  Execute tool with the following command.<br>

  ruby -Itool_path mem_layout.conf layout_header fence_header pool_header<br>

  Ex) ruby -Isony_apps/memutils/memory_manager/tool sony_apps/examples/audio/audio_player/mem_layout.conf mem_layout.h fixed_fence.h pool_layout.h<br>

  |Parameter      | Description                                                                            |
  |:--------------|:---------------------------------------------------------------------------------------|
  |-Itool_path    | path of mem_layout.rb                                                                  |
  |mem_layout.conf| memory layout configuration file for memory manager                                    |
  |layout_header  | file to which various contant valus definition is output                               |
  |fence_header   | file to which memory fence address of fixed area is output(only used by memory manager)|
  |pool_header    | file to which various difinitions of pool area is output(only used by memory manager)  |
 
  ## Sequence # {#spritzer_memory_manager_api_sequence}

  Call APIs according to the following sequence diagram.
 ![sequence diagram](memory_manager_api_sequence.png)

  ## Appendix # {#spritzer_memory_manager_appendix_1}

  [japanese instruction manual](../../sony_apps/memutils/memory_manager/doc/MemMgrLite.txt "MemMgrLite")

---------------------

# Message Library # {#spritzer_message}

  ## General # {#spritzer_message_general}

  This "Message Library" can send and receive class objects each inter-tasks.<br>


  ## APIs # {#spritzer_message_api}
  - @ref memutils_message

  ## Configurations # {#spritzer_massage_config}

  "Massage" can be configurations sender & receiver by users configuration and their tools.
  
  ### How to the use tool # {#spritzer_massage_config_how_to}

  Execute tool with the following command.<br>

  ruby -Itool_path msgq_layout.conf start_addr size id_header pool_header<br>

  Ex) ruby -Isony_apps/memutils/message/tool sony_apps/examples/audio/audio_player/config/msgq_layout.conf 0x000fc000 0x3140 msgq_id.h msgq_pool.h<br>

  |Parameter       | Description                                             |
  |:---------------|:--------------------------------------------------------|
  |-Itool_path     | path of msgq_layout.rb                                  |
  |msgq_layout.conf| sender & receiver configuration file for message library|
  |start_addr      | address of message area                                 |
  |size            | size of message area                                    |
  |id_header       | file to which message queue ID macro is output          |
  |pool_header     | file to which message queue pool definition is output   |

  ## Sequence # {#spritzer_message_api_sequence}

  Call APIs according to the following sequence diagram.
  ![sequence diagram](message_api_sequence.png)

  ## Appendix # {#spritzer_memory_manager_appendix_1}
  
  [japanese instruction manual](../../sony_apps/memutils/message/doc/MessageLib200.txt "MessageLib")


---------------------

# Simple FIFO #   {#spritzer_simpleFIFO}

  ## General # {#spritzer_simpleFIFO_general}

This library supports one-writer and one-reader access without any exclusive control.
 Some exclusive access control is required to support multi-writer/reader outside of this library.

This library supports access from multi-processor inserting proper data-sync-barriers and data-memory-barriers. 
It makes sure the order of update data contents and WP/RP for the purpose.

  ## APIs # {#spritzer_simpleFIFO_api}
  - @ref memuils_simple_fifo


---------------------

# Integrated Sequence #   {#spritzer_memutil_sequence}

The general sequence how to use with "memory manager" & "message library" be shown below.

![sequence diagram](memory_manager_intergrated_sequence.png)

The general sequence how to use with "Simple FIFO" be shown below.


# Configurrations & builds#     {#spritzer_memutil_build}


# Code Examples #   {#spritzer_memutil_example}

These libraries are used with audio features.
So, their samples are audio samples.


example sony_apps/examples/audio/audio_player <br>
example sony_apps/examples/audio/audio_recoder <br>
example sony_apps/examples/audio/voice_call <br>
example sony_apps/examples/audio/voice_command <br>
