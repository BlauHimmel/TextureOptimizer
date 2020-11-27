#include "Texture.hpp"

#include <filesystem>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

bool Texture::Create(uint32_t width, uint32_t height, uint32_t channel)
{
    if (width == 0 || height == 0 || channel == 0 || channel > 4)
    {
        return false;
    }

    m_bValid = true;
    m_Width = width;
    m_Height = height;
    m_Channels = channel;
    m_Data.reset(new uint8_t[width * height * channel]);

    return true;
}

bool Texture::Load(const std::string & filename)
{
    int x, y, n;
    stbi_uc * pData = stbi_load(filename.c_str(), &x, &y, &n, 0);
    if (pData == nullptr)
    {
        return false;
    }

    m_bValid = true;
    m_Width = static_cast<uint32_t>(x);
    m_Height = static_cast<uint32_t>(y);
    m_Channels = static_cast<uint32_t>(n);
    m_Data.reset(pData);

    return true;
}

bool Texture::Write(const std::string & filename) const
{
    if (!m_bValid) { return false; }

    std::filesystem::path path(filename);
    std::string ext = path.extension().string();

    int w = static_cast<int>(m_Width);
    int h = static_cast<int>(m_Height);
    int comp = static_cast<int>(m_Channels);
    const void * data = m_Data.get();

    if (ext == ".png")
    {
        return stbi_write_png(filename.c_str(), w, h, comp, data, 0) != 0;
    }
    if (ext == ".bmp")
    {
        return stbi_write_bmp(filename.c_str(), w, h, comp, data) != 0;
    }
    if (ext == ".tga")
    {
        return stbi_write_tga(filename.c_str(), w, h, comp, data) != 0;
    }
    if (ext == ".jpg")
    {
        return stbi_write_jpg(filename.c_str(), w, h, comp, data, 100) != 0;
    }

    return false;
}

std::unique_ptr<Texture> Texture::Copy() const
{
    std::unique_ptr<Texture> texture(new Texture);
    texture->m_bValid = m_bValid;
    texture->m_Channels = m_Channels;
    texture->m_Width = m_Width;
    texture->m_Height = m_Height;
    texture->m_Data.reset(new uint8_t[m_Width * m_Height * m_Channels]);
    memcpy(texture->m_Data.get(), m_Data.get(), sizeof(uint8_t) * m_Width * m_Height * m_Channels);
    return texture;
}

void Texture::CopyTo(Texture * pTexture) const
{
    pTexture->m_bValid = m_bValid;
    pTexture->m_Channels = m_Channels;
    pTexture->m_Width = m_Width;
    pTexture->m_Height = m_Height;
    pTexture->m_Data.reset(new uint8_t[m_Width * m_Height * m_Channels]);
    memcpy(pTexture->m_Data.get(), m_Data.get(), sizeof(uint8_t) * m_Width * m_Height * m_Channels);
}

uint8_t & Texture::At(uint32_t x, uint32_t y, uint32_t channels) const
{
    return *Fetch(this, x, y, channels);
}

uint8_t * Texture::At(uint32_t x, uint32_t y) const
{
    return Fetch(this, x, y, 0);
}

std::unique_ptr<uint8_t[]> Texture::Sample(double x, double y) const
{
    auto x0 = static_cast<uint32_t>(std::floor(x * (m_Width  - 1)));
    auto y0 = static_cast<uint32_t>(std::floor(y * (m_Height - 1)));
    auto x1 = x0 + 1;
    auto y1 = y0 + 1;

    x0 = std::clamp(x0, 0u, m_Width  - 1u);
    x1 = std::clamp(x1, 0u, m_Width  - 1u);
    y0 = std::clamp(y0, 0u, m_Height - 1u);
    y1 = std::clamp(y1, 0u, m_Height - 1u);

    std::unique_ptr<uint8_t[]> pixel(new uint8_t[m_Channels]);

    uint8_t * pX0Y0 = Fetch(this, x0, y0, 0);
    uint8_t * pX0Y1 = Fetch(this, x0, y1, 0);
    uint8_t * pX1Y0 = Fetch(this, x1, y0, 0);
    uint8_t * pX1Y1 = Fetch(this, x1, y1, 0);

    for (uint32_t channel = 0; channel < m_Channels; channel++)
    {
        uint32_t sum = static_cast<uint32_t>(pX0Y0[channel]) +
                       static_cast<uint32_t>(pX0Y1[channel]) +
                       static_cast<uint32_t>(pX1Y0[channel]) +
                       static_cast<uint32_t>(pX1Y1[channel]);
        pixel[channel] = static_cast<uint8_t>(sum / 4);
    }

    return pixel;
}

bool Texture::IsValid() const
{
    return m_bValid;
}

uint32_t Texture::Width() const
{
    return m_Width;
}

uint32_t Texture::Height() const
{
    return m_Height;
}

uint32_t Texture::Channels() const
{
    return m_Channels;
}

std::unique_ptr<Texture> Texture::CreateChessBoard(uint32_t width, uint32_t height, uint32_t xCount, uint32_t yCount)
{
    if (width == 0 || height == 0 || xCount == 0 || yCount == 0)
    {
        return nullptr;
    }

    uint32_t xIncre = width / xCount;
    uint32_t yIncre = height / yCount;

    if (xIncre == 0 || yIncre == 0)
    {
        return nullptr;
    }

    auto texture = std::make_unique<Texture>();
    texture->Create(width, height, 3);

    uint32_t idx = 0;
    for (uint32_t x = 0; x < width; x += xIncre)
    {
        for (uint32_t y = 0; y < width; y += yIncre)
        {
            uint8_t val = (idx % 2 == 0 ? 255 : 0);

            for (uint32_t xx = 0; xx < xIncre && x + xx < width; xx++)
            {
                for (uint32_t yy = 0; yy < yIncre && y + yy < height; yy++)
                {
                    texture->At(x + xx, y + yy, 0) = val;
                    texture->At(x + xx, y + yy, 1) = val;
                    texture->At(x + xx, y + yy, 2) = val;
                }
            }
            idx++;
        }
        idx++;
    }

    return texture;
}

uint8_t * Texture::Fetch(const Texture * pTexture, uint32_t x, uint32_t y, uint32_t channels)
{
    assert(pTexture != nullptr && pTexture->m_bValid && x < pTexture->m_Width && y < pTexture->m_Height && channels < pTexture->m_Channels);
    return pTexture->m_Data.get() + ((y * pTexture->m_Width + x) * pTexture->m_Channels + channels);
}