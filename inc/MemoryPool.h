#pragma once
#include "SIPPNode.h"

const int DEFAULT_CHUNK_SIZE = 25000;

class MemoryChunk
{
public:
    MemoryChunk() : nodes(nullptr){};

    inline void
    allocate()
    {
        cout << "allocating nodes" << endl;
        nodes = new SIPPNode[DEFAULT_CHUNK_SIZE];
    }

    SIPPNode *get_node(int id)
    {
        if (nodes == nullptr)
            allocate();

        if (id >= DEFAULT_CHUNK_SIZE)
        {
            std::cout << "range out of chunk size " << id << "," << DEFAULT_CHUNK_SIZE << std::endl;
            exit(1);
        }

        return &nodes[id];
    }

    inline bool
    is_allocated()
    {
        return nodes != nullptr;
    }

    ~MemoryChunk()
    {
        delete[] nodes;
    }

private:
    SIPPNode *nodes;
};

class MemoryPool
{
    // every location in the map has a node with id equal to the location
public:
    MemoryPool()
    {
        numChunks = 0;
        size = 0;
        index = 0;
        chunks = nullptr;
        label = 0;
    };
    MemoryPool(int size)
    {
        init(size);
    };

    int generated()
    {
        return index;
    }

    void init(int size)
    {
        this->size = size;
        index = 0;
        label = 0;
        numChunks = (size / DEFAULT_CHUNK_SIZE) + 1;
        chunks = new MemoryChunk[numChunks];
        ready = true;
    }

    bool is_ready()
    {
        return ready;
    }

    bool has_node(int id)
    {
        if (id >= size)
        {
            std::cout << "range out of memory pool size " << id << "," << index << "," << size << std::endl;
            exit(1);
        }
        SIPPNode *node = get_node_(id);
        return node->label == label && node->id == id;
    }
    bool is_closed(int id)
    {
        if (id >= size)
        {
            std::cout << "range out of memory pool size " << id << "," << index << "," << size << std::endl;
            exit(1);
        }
        SIPPNode *node = get_node_(id);
        if (node->label != label)
        {
            return false;
        }
        return node->is_closed;
    }
    SIPPNode *get_node(int id)
    {
        if (id >= size)
        {
            std::cout << "range out of memory pool size " << id << "," << index << "," << size << std::endl;
            exit(1);
        }
        SIPPNode *node = get_node_(id);
        if (node->label != label || node->id == -1)
        {
            std::cout << "error node not generated yet" << std::endl;
            exit(1);
        }
        return node;
    }
    void close_node(int id)
    {
        if (id >= size)
        {
            std::cout << "range out of memory pool size " << id << "," << index << "," << size << std::endl;
            exit(1);
        }
        SIPPNode *node = get_node_(id);
        if (node->label != label || node->id == -1)
        {
            std::cout << "node not generated yet" << std::endl;
            exit(1);
        }
        node->close();
    }
    SIPPNode *generate_node(int id, int location, int g_val, int h_val, SIPPNode *parent, int timestep, int high_generation, int high_expansion,
                            bool collision_v, int num_of_conflicts)
    {

        if (id >= size)
        {
            std::cout << "range out of memory pool size " << id << "," << index << "," << size << std::endl;
            exit(1);
        }

        SIPPNode *node = get_node_(id);
        if (node->label == label && node->id != -1)
        {
            std::cout << "node already generated " << id << "," << is_ready() << std::endl;

            std::cout << "node already generated " << node->id << std::endl;
            exit(1);
        }
        node->reset();
        node->id = id;
        node->label = label;
        node->location = location;
        node->g_val = g_val;
        node->h_val = h_val;
        node->parent = parent;
        node->timestep = timestep;
        node->high_generation = high_generation;
        node->high_expansion = high_expansion;
        node->collision_v = collision_v;
        node->num_of_conflicts = num_of_conflicts;

        index++;
        return node;
    }

    void free_node(int id)
    {
        if (id >= size)
        {
            std::cout << "range out of memory pool size " << id << "," << index << "," << size << std::endl;
            exit(1);
        }
        SIPPNode *node = get_node_(id);
        if (node->id == -1)
        {
            std::cout << "node not generated yet" << std::endl;
            exit(1);
        }
        node->reset();
        index--;
    }

    SIPPNode *replace_node(int id, SIPPNode &new_node)
    {
        SIPPNode *node = get_node_(id);
        node->reset();
        node->id = id;
        node->label = label;
        node->location = new_node.location;
        node->g_val = new_node.g_val;
        node->h_val = new_node.h_val;
        node->parent = new_node.parent;
        node->timestep = new_node.timestep;
        node->high_generation = new_node.high_generation;
        node->high_expansion = new_node.high_expansion;
        node->collision_v = new_node.collision_v;
        node->num_of_conflicts = new_node.num_of_conflicts;
        return node;
    }

    void reset()
    {
        index = 0;
        label++;
    }

    ~MemoryPool()
    {
        // for bucket in nodes
        // if bucket is not nullp
        // delete bucket
        // delete bucket ptr
        MemoryChunk *ptr = &chunks[0];
        for (int i = 0; i < numChunks; i++)
        {
            if (ptr->is_allocated())
            {
                delete[] ptr;
                ptr = nullptr;
                ptr++;
            }
        }
    }

private:
    MemoryChunk *chunks;
    int numChunks;
    int size;
    int index;
    int label;
    bool ready = false;

    // find bucket id and allocate if does not exist.
    SIPPNode *get_node_(int node_id)
    {

        int chunk_id = node_id / DEFAULT_CHUNK_SIZE;
        int internal_id = node_id % DEFAULT_CHUNK_SIZE;
        // std::cout << node_id << "," << chunk_id << "," << internal_id << endl;

        if (chunk_id >= numChunks)
        {
            std::cout << "range out of memory pool size " << chunk_id << "," << index << "," << size << std::endl;
            exit(1);
        }

        return chunks[chunk_id].get_node(internal_id);
    }
};
