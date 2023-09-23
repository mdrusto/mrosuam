#pragma once

#include <imgui.h>

struct ScrollingBuffer
{
    int m_maxSize;
    int m_offset;
    ImVector<ImVec2> m_data;
    
    ScrollingBuffer(int max_size = 1000)
    {
        m_maxSize = max_size;
        m_offset = 0;
        m_data.reserve(m_maxSize);
    }
    
    void addPoint(float x, float y)
    {
        if (m_data.size() < m_maxSize)
            m_data.push_back(ImVec2(x, y));
        else
        {
            m_data[m_offset] = ImVec2(x, y);
            m_offset = (m_offset + 1) % m_maxSize;
        }
    }
    
    void erase()
    {
        if (m_data.size() > 0)
        {
            m_data.shrink(0);
            m_offset = 0;
        }
    }
};
