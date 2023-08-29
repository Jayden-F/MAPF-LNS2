#pragma once
#include "SIPPNode.h"

class MemoryPool
{
    // every location in the map has a node with id equal to the location
public:
    MemoryPool()
    {
        size = 0;
        index = 0;
        nodes = nullptr;
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
        nodes = new SIPPNode[size];
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
        return nodes[id].label == label && nodes[id].id == id;
    }
    bool is_closed(int id)
    {
        if (id >= size)
        {
            std::cout << "range out of memory pool size " << id << "," << index << "," << size << std::endl;
            exit(1);
        }
        if (nodes[id].label != label)
        {
            return false;
        }
        return nodes[id].is_closed;
    }
    SIPPNode *get_node(int id)
    {
        if (id >= size)
        {
            std::cout << "range out of memory pool size " << id << "," << index << "," << size << std::endl;
            exit(1);
        }
        if (nodes[id].label != label || nodes[id].id == -1)
        {
            std::cout << "error node not generated yet" << std::endl;
            exit(1);
        }
        return &(nodes[id]);
    }
    void close_node(int id)
    {
        if (id >= size)
        {
            std::cout << "range out of memory pool size " << id << "," << index << "," << size << std::endl;
            exit(1);
        }
        if (nodes[id].label != label || nodes[id].id == -1)
        {
            std::cout << "node not generated yet" << std::endl;
            exit(1);
        }
        nodes[id].close();
    }
    SIPPNode *generate_node(int id, int location, int g_val, int h_val, SIPPNode *parent, int timestep, int high_generation, int high_expansion,
                            bool collision_v, int num_of_conflicts)
    {

        if (id >= size)
        {
            std::cout << "range out of memory pool size " << id << "," << index << "," << size << std::endl;
            exit(1);
        }

        if (nodes[id].label == label && nodes[id].id != -1)
        {
            std::cout << "node already generated " << id << "," << is_ready() << std::endl;

            std::cout << "node already generated " << nodes[id].id << std::endl;
            exit(1);
        }
        nodes[id].reset();
        nodes[id].id = id;
        nodes[id].label = label;
        nodes[id].location = location;
        nodes[id].g_val = g_val;
        nodes[id].h_val = h_val;
        nodes[id].parent = parent;
        nodes[id].timestep = timestep;
        nodes[id].high_generation = high_generation;
        nodes[id].high_expansion = high_expansion;
        nodes[id].collision_v = collision_v;
        nodes[id].num_of_conflicts = num_of_conflicts;

        index++;
        return &(nodes[id]);
    }

    void free_node(int id)
    {
        if (id >= size)
        {
            std::cout << "range out of memory pool size " << id << "," << index << "," << size << std::endl;
            exit(1);
        }
        if (nodes[id].id == -1)
        {
            std::cout << "node not generated yet" << std::endl;
            exit(1);
        }
        nodes[id].reset();
        index--;
    }

    SIPPNode *replace_node(int id, SIPPNode &node)
    {

        nodes[id] = node;
        nodes[id].id = id;
        nodes[id].label = label;
        return &(nodes[id]);
    }

    void reset()
    {
        index = 0;
        label++;
    }

    ~MemoryPool()
    {
        if (nodes != nullptr)
        {
            delete[] nodes;
            nodes = nullptr;
        }
    }

private:
    SIPPNode *nodes;
    int size;
    int index;
    int label;
    bool ready = false;
};
