#ifndef VERSIONCHECKER_H
#define VERSIONCHECKER_H

#include <QObject>
#include <QNetworkAccessManager>
#include <QVersionNumber>

class VersionChecker : public QObject
{
    Q_OBJECT

public:
    explicit VersionChecker(QObject *parent = nullptr);
    ~VersionChecker();

    /// Check if a newer version is available
    /// @param currentVersion The current version of the application
    /// @param repoOwner GitHub repository owner (e.g., "SLU-Agrosystem")
    /// @param repoName GitHub repository name (e.g., "RControlStation")
    void checkForUpdates(const QVersionNumber &currentVersion, 
                         const QString &repoOwner = "SLU-Agrosystem",
                         const QString &repoName = "RControlStation");

    /// Get the latest version found
    QVersionNumber getLatestVersion() const;

    /// Get the URL to download the latest version
    QString getDownloadUrl() const;

    /// Check if an update is available
    bool isUpdateAvailable() const;

    /// Get the release notes for the latest version
    QString getReleaseNotes() const;

signals:
    /// Emitted when update check is complete
    void updateCheckFinished(bool updateAvailable, 
                             const QVersionNumber &latestVersion,
                             const QString &downloadUrl,
                             const QString &releaseNotes);

    /// Emitted when an error occurs during update check
    void updateCheckError(const QString &errorMessage);

private slots:
    void onReleasesFetched();
    void onReleaseNotesFetched();

private:
    QNetworkAccessManager *m_networkManager;
    QVersionNumber m_latestVersion;
    QString m_downloadUrl;
    QString m_releaseNotes;
    bool m_updateAvailable;
    QString m_repoOwner;
    QString m_repoName;
    QVersionNumber m_currentVersion;

    /// Parse GitHub API response to find the latest version
    void parseGitHubReleases(const QByteArray &data);
};

#endif // VERSIONCHECKER_H
