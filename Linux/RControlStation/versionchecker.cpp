#include "versionchecker.h"
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QVersionNumber>
#include <QDebug>

VersionChecker::VersionChecker(QObject *parent) 
    : QObject(parent),
      m_networkManager(new QNetworkAccessManager(this)),
      m_updateAvailable(false)
{
    connect(m_networkManager, &QNetworkAccessManager::finished, 
            this, &VersionChecker::onReleasesFetched);
}

VersionChecker::~VersionChecker()
{
    // m_networkManager is deleted automatically (parent-child relationship)
}

void VersionChecker::checkForUpdates(const QVersionNumber &currentVersion,
                                     const QString &repoOwner,
                                     const QString &repoName)
{
    m_currentVersion = currentVersion;
    m_repoOwner = repoOwner;
    m_repoName = repoName;
    m_updateAvailable = false;
    m_releaseNotes.clear();
    m_downloadUrl.clear();

    // GitHub API URL for releases
    QString apiUrl = QString("https://api.github.com/repos/%1/%2/releases/latest")
            .arg(repoOwner, repoName);

    QNetworkRequest request(apiUrl);
    // GitHub API requires a user agent
    request.setRawHeader("User-Agent", "RControlStation");
    
    // Accept JSON response
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    
    m_networkManager->get(request);
}

QVersionNumber VersionChecker::getLatestVersion() const
{
    return m_latestVersion;
}

QString VersionChecker::getDownloadUrl() const
{
    return m_downloadUrl;
}

bool VersionChecker::isUpdateAvailable() const
{
    return m_updateAvailable;
}

QString VersionChecker::getReleaseNotes() const
{
    return m_releaseNotes;
}

void VersionChecker::onReleasesFetched()
{
    QNetworkReply *reply = qobject_cast<QNetworkReply*>(sender());
    
    if (!reply) {
        emit updateCheckError("Network error: Invalid reply");
        return;
    }

    if (reply->error() != QNetworkReply::NoError) {
        QString errorMsg = QString("Network error: %1").arg(reply->errorString());
        // Check if it's a 404 (repo not found) or rate limit
        if (reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt() == 404) {
            errorMsg = "Repository not found. Please check the repository settings.";
        } else if (reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt() == 403) {
            errorMsg = "GitHub API rate limit exceeded. Please try again later.";
        }
        emit updateCheckError(errorMsg);
        reply->deleteLater();
        return;
    }

    QByteArray data = reply->readAll();
    reply->deleteLater();

    parseGitHubReleases(data);
}

void VersionChecker::parseGitHubReleases(const QByteArray &data)
{
    QJsonDocument doc = QJsonDocument::fromJson(data);
    
    if (!doc.isObject()) {
        emit updateCheckError("Failed to parse GitHub API response");
        return;
    }

    QJsonObject release = doc.object();
    
    // Extract version tag (e.g., "v1.2.3" -> "1.2.3")
    QString tagName = release.value("tag_name").toString();
    if (tagName.isEmpty()) {
        emit updateCheckError("No version tag found in release");
        return;
    }

    // Remove 'v' prefix if present
    if (tagName.startsWith('v') || tagName.startsWith('V')) {
        tagName = tagName.mid(1);
    }

    QVersionNumber latestVersion = QVersionNumber::fromString(tagName);
    if (!latestVersion.isNull() && latestVersion > m_currentVersion) {
        m_latestVersion = latestVersion;
        m_updateAvailable = true;
        
        // Get download URL (try to find a .deb, .exe, or .tar.gz)
        QJsonArray assets = release.value("assets").toArray();
        for (const QJsonValue &assetVal : assets) {
            QJsonObject asset = assetVal.toObject();
            QString assetName = asset.value("name").toString();
            QString browserDownloadUrl = asset.value("browser_download_url").toString();
            
            // Prefer platform-specific packages, but store the first one we find
            if (m_downloadUrl.isEmpty()) {
                m_downloadUrl = browserDownloadUrl;
            }
            
            // For Linux, prefer .deb packages
            if (assetName.endsWith(".deb")) {
                m_downloadUrl = browserDownloadUrl;
                break; // Prefer .deb for Linux
            }
            // For Windows, prefer .exe installers
            else if (assetName.endsWith(".exe")) {
                m_downloadUrl = browserDownloadUrl;
                // Don't break, .deb might still be found
            }
        }
        
        // Get release notes
        m_releaseNotes = release.value("body").toString();
        
        emit updateCheckFinished(true, m_latestVersion, m_downloadUrl, m_releaseNotes);
    } else {
        // No update available
        m_updateAvailable = false;
        emit updateCheckFinished(false, latestVersion, "", "");
    }
}

void VersionChecker::onReleaseNotesFetched()
{
    // This slot is reserved for future use if we need to fetch release notes separately
    QNetworkReply *reply = qobject_cast<QNetworkReply*>(sender());
    if (reply) {
        reply->deleteLater();
    }
}
